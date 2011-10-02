// RFM12B driver implementation
// 2009-02-09 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: RF12.cpp 7715 2011-06-09 17:49:45Z jcw $

#include "RF12RKR.h"
#include <avr/io.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <WProgram.h>

// maximum transmit / receive buffer: 3 header + data + 2 crc bytes

// pins used for the RFM12B interface
#if defined(__AVR_ATmega1280__)

#define RFM_IRQ     2
#define SS_PORT     PORTB
#define SS_BIT      0
#define SPI_SS      53
#define SPI_MOSI    51
#define SPI_MISO    50
#define SPI_SCK     52

#elif defined(__AVR_ATtiny84__)

#define RFM_IRQ     2
#define SS_PORT     PORTA
#define SS_BIT      7
#define SPI_SS      3   // PA7, pin 6
#define SPI_MISO    4   // PA6, pin 7
#define SPI_MOSI    5   // PA5, pin 8
#define SPI_SCK     6   // PA4, pin 9

#else

// ATmega328, etc.
#define RFM_IRQ     2
#define SS_PORT     PORTB
#define SS_BIT      2
#define SPI_SS      10
#define SPI_MOSI    11
#define SPI_MISO    12
#define SPI_SCK     13

#endif

// RF12 command codes
#define RF_RECEIVER_ON  0x82DD
#define RF_XMITTER_ON   0x823D
#define RF_IDLE_MODE    0x820D
#define RF_SLEEP_MODE   0x8205
#define RF_WAKEUP_MODE  0x8207
#define RF_TXREG_WRITE  0xB800
#define RF_RX_FIFO_READ 0xB000
#define RF_WAKEUP_TIMER 0xE000

// RF12 status bits
#define RF_LBD_BIT      0x0400
#define RF_RSSI_BIT     0x0100

// bits in the node id configuration byte
#define NODE_BAND       0xC0        // frequency band
#define NODE_ACKANY     0x20        // ack on broadcast packets if set
#define NODE_ID         0x1F        // id of this node, as A..Z or 1..31

// transceiver states, these determine what to do with each interrupt
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
    TXRPRE1, TXRPRE2, TXRPRE3, TXRSYN1, TXRSYN2, TXRECV,
    TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

static uint8_t nodeid;              // address of this node
static uint8_t group;               // network group
uint8_t rf12_len;     // number of data bytes in rf12_buf
static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
static volatile int8_t rxstate;     // current transceiver state

#define RETRIES     8               // stop retrying after 8 times
#define RETRY_MS    1000            // resend packet every second until ack'ed

static uint8_t ezInterval;          // number of seconds between transmits
static uint8_t ezSendBuf[RF12_MAXDATA]; // data to send
static char ezSendLen;              // number of bytes to send
static uint8_t ezPending;           // remaining number of retries
static long ezNextSend[2];          // when was last retry [0] or data [1] sent

volatile uint16_t rf12_status;         // running crc value
volatile uint8_t rf12_buf[RF_MAX];  // recv/xmit buf, including hdr & crc bytes
long rf12_seq;                      // seq number of encrypted packet (or -1)

static uint32_t freqTimeout = 0;
static unsigned long recvTimeout = 0;
static unsigned long recvStart = 0;

static uint32_t seqNum;             // encrypted send sequence number
static uint32_t cryptKey[4];        // encryption key to use
void (*crypter)(uint8_t);           // does en-/decryption (null if disabled)

static void spi_initialize () {
    digitalWrite(SPI_SS, 1);
    pinMode(SPI_SS, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_SCK, OUTPUT);
#ifdef SPCR
#if F_CPU <= 10000000
    // clk/4 is ok for the RF12's SPI
    SPCR = _BV(SPE) | _BV(MSTR);
#else
    // use clk/8 (2x 1/16th) to avoid exceeding RF12's SPI specs of 2.5 MHz
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
    SPSR |= _BV(SPI2X);
#endif
#else
    // ATtiny
    USICR = bit(USIWM0);
#endif
}

static uint8_t rf12_byte (uint8_t out) {
#ifdef SPDR
    SPDR = out;
    // this loop spins 4 usec with a 2 MHz SPI clock
    while (!(SPSR & _BV(SPIF)))
        ;
    return SPDR;
#else
    // ATtiny
    USIDR = out;
    byte v1 = bit(USIWM0) | bit(USITC);
    byte v2 = bit(USIWM0) | bit(USITC) | bit(USICLK);
#if F_CPU <= 5000000
    // only unroll if resulting clock stays under 2.5 MHz
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
#else
    for (uint8_t i = 0; i < 8; ++i) {
        USICR = v1;
        USICR = v2;
    }
#endif
    return USIDR;
#endif
}

static uint16_t rf12_xfer (uint16_t cmd) {
    bitClear(SS_PORT, SS_BIT);
    uint16_t reply = rf12_byte(cmd >> 8) << 8;
    reply |= rf12_byte(cmd);
    bitSet(SS_PORT, SS_BIT);
    return reply;
}

// access to the RFM12B internal registers with interrupts disabled
uint16_t rf12_control(uint16_t cmd) {
#ifdef EIMSK
    bitClear(EIMSK, INT0);
    uint16_t r = rf12_xfer(cmd);
    bitSet(EIMSK, INT0);
#else
    // ATtiny
    bitClear(GIMSK, INT0);
    uint16_t r = rf12_xfer(cmd);
    bitSet(GIMSK, INT0);
#endif
    return r;
}

/*
	17. Status Read Command
	The read command starts with a zero, whereas all other control commands start with a one. If a read command is identified, the
	status bits will be clocked out on the SDO pin as follows:

	bitnumber
	15	RGIT TX register is ready to receive the next byte (Can be cleared by Transmitter Register Write Command)
		FFIT The number of data bits in the RX FIFO has reached the pre-programmed limit (Can be cleared by any of the
		FIFO read methods)
	14	POR Power-on reset (Cleared after Status Read Command)
	13	RGUR TX register under run, register over write (Cleared after Status Read Command)
		FFOV RX FIFO overflow (Cleared after Status Read Command)
	12	WKUP Wake-up timer overflow (Cleared after Status Read Command)

	11	EXT Logic level on interrupt pin (pin 16) changed to low (Cleared after Status Read Command)
	10	LBD Low battery detect, the power supply voltage is below the pre-programmed limit
	9	FFEM FIFO is empty
	8	ATS Antenna tuning circuit detected strong enough RF signal
		RSSI The strength of the incoming signal is above the pre-programmed limit

	7	DQD Data quality detector output
	6	CRL Clock recovery locked
	5	ATGL Toggling in each AFC cycle
	4	OFFS(6) MSB of the measured frequency offset (sign of the offset value)

	3	OFFS(3) -OFFS(0) Offset value to be added to the value of the frequency control parameter (Four LSB bits)
	2
	1
	0
*/

#define RFM12_CMD_STATUS 	0x0000
#define RFM12_STATUS_RGIT 	0x8000
#define RFM12_STATUS_FFIT 	0x8000
#define RFM12_STATUS_POR 	0x4000
#define RFM12_STATUS_RGUR 	0x2000
#define RFM12_STATUS_FFOV 	0x2000
#define RFM12_STATUS_WKUP 	0x1000
#define RFM12_STATUS_EXT 	0x0800
#define RFM12_STATUS_LBD 	0x0400
#define RFM12_STATUS_FFEM 	0x0200
#define RFM12_STATUS_ATS 	0x0100
#define RFM12_STATUS_RSSI 	0x0100
#define RFM12_STATUS_DQD 	0x0080
#define RFM12_STATUS_CRL 	0x0040
#define RFM12_STATUS_ATGL 	0x0020

#ifdef SOFT_SYNC
static void rf12_interruptsoftsync() {
    // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
    rf12_status = rf12_xfer(0x0000);

    if ((rxstate >= TXRPRE1 && rxstate <= TXRECV) && ((rf12_status & RFM12_STATUS_FFEM) == 0)) {
        uint8_t in = rf12_xfer(RF_RX_FIFO_READ);
		if (rxfill < RF_MAX) {
			rf12_buf[rxfill++] = in;
		}
		else {
			rf12_xfer(RF_IDLE_MODE);
		}
		if (((in == 0xAA) || (in == 0x55))) {
			if (rxstate == TXRECV) {
#ifdef SYNC2
					if (rxfill >= RF_MAX2 && in == rf12_buf[rxfill - 2] && in == rf12_buf[rxfill - 3]) {
						rf12_buf[0] += 1;
					}
#else
					if (rxfill >= RF_MAX2 && in == rf12_buf[rxfill - 2]) {
						rf12_buf[0] += 1;
					}
#endif
			}
			else if ((in == rf12_buf[RXFILL_MIN]) && (rxfill > (RXFILL_MIN + RXFILL_SYNC))) { // SYNC
        		rf12_buf[0] += 1;
        		rxstate = TXRECV;
			}
		}
		else if (rxstate != TXRECV) {
			rf12_buf[0] = 0;
			rxfill = RXFILL_MIN;
			rf12_buf[rxfill] = 0;
		}
   }
   else if ((rxstate == TXRECV) && ((rf12_status & RFM12_STATUS_FFOV) != 0)) {
	   rf12_buf[1] += 1;
   }
   else if ((rxstate == TXRECV) && ((rf12_status & RFM12_STATUS_FFEM) != 0)) {
	   rf12_buf[2] += 1;
   }
}
#endif

static void rf12_interrupt() {
    // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
    rf12_status = rf12_xfer(0x0000);
    if (rxstate != TXRECV) {
		return;
	}

//    if ((rf12_status & RFM12_STATUS_FFEM) == 0) {
    if ((rf12_status & RFM12_STATUS_FFIT) != 0) {
        uint8_t in = rf12_xfer(RF_RX_FIFO_READ);

        //if (rxfill == 0 && group != 0)
        //    rf12_buf[rxfill++] = group;
		if (rxfill < RF_MAX2) {
        	rf12_buf[rxfill++] = in;
	        //rf12_status = _crc16_update(rf12_status, in);
		}
		if (rxfill >= RF_MAX2 + 10) {
		    //rf12_xfer(0xCA89); // FIFO8,1-SYNC,!ff,DR
#if 0
            rf12_xfer(RF_IDLE_MODE);
#else
			rf12_xfer(0xCA81); // Almost CE on RFM01 FIFO8,1-SYNC,!ff,DR -- 7 no sync...
			rf12_xfer(0xCA83); // Almost CE on RFM01 FIFO8,1-SYNC,!ff,DR -- 7 no sync...
#endif
		}
	}

   if ((rf12_status & RFM12_STATUS_FFOV) != 0) {
	   rf12_buf[1] += 1;
   }
   if ((rf12_status & RFM12_STATUS_FFIT) != 0) {
	   rf12_buf[2] += 1;
   }
   if ((rf12_status & RFM12_STATUS_FFEM) != 0) {
	   rf12_buf[0] += 1;
   }
}


static void rf12_recvStart () {
//	Serial.print(millis());
//	Serial.print(' ');
//	Serial.print(freqTimeout);
//	Serial.print(' ');
#if 0
	if (millis() >= freqTimeout) {
		rxstate = TXIDLE;
		if ((freqTimeout == 0) || (freq > 0xA700)) {
			freq = 0xA600; // min, max and step in this procedure
		}
		else {
			freq += 0x10;
		}
		//rf12_initialize(0, 0, 0);
		rf12_xfer(RF_IDLE_MODE);
		rf12_xfer(freq); // + 4 experiment

		Serial.print("Change frequency: ");
		Serial.println(freq, HEX);
		freqTimeout = millis();
		freqTimeout += 3*60*1000UL;
	}
#endif
#if 1
	if (millis() >= freqTimeout) {
		rxstate = TXIDLE;
		rf12_xfer(RF_IDLE_MODE);
		//rf12_initialize(0, 0, 0);
		rf12_config(0);
		rf12_xfer(RF_IDLE_MODE);
		freqTimeout = millis();
		freqTimeout += 5*60*1000UL;
	}
#endif
//    recvTimeout =millis() + 0;
   rxfill = rf12_len = RXFILL_MIN;
    rf12_status = ~0;
#if RF12_VERSION >= 2
    if (group != 0)
        rf12_status = _crc16_update(~0, group);
#endif
	// flush fifo
//    rf12_xfer(0xCA89); // FIFO8,1-SYNC,!ff,DR
//    rf12_xfer(0xCA8B); // FIFO8,1-SYNC,!ff,DR
	memset((void*)rf12_buf, 0, sizeof(*rf12_buf) * RF_MAX);
#ifdef SOFT_SYNC
	rxstate = TXRPRE1;
#else
    rxstate = TXRECV;
#endif
    //Serial.println('S');
    rf12_xfer(RF_RECEIVER_ON);
    recvTimeout =millis() + 30000;
    recvStart = millis();
}
void PrintTerm()
  {
;
  }

void PrintChar(byte S)
  {
  Serial.print(S, BYTE);
  }
void PrintDash(void)
  {
  PrintChar('-');
  }

void PrintComma(void)
  {
  Serial.print(", ");
  }

void PrintNum(uint x, char c, uint digits) {
     // Rinie add space for small digits
     if(c) {
     	PrintChar(c);
 	}
	for (uint i=1, val=10; i < digits; i++, val *= 10) {
		if (x < val) {
			PrintChar(' ');
		}
	}

    Serial.print(x,DEC);
}

void PrintNumHex(uint x, char c, uint digits) {
	// Rinie add space for small digits
	if(c) {
		PrintChar(c);
	}
	for (uint i=1, val=16; i < digits; i++, val *= 16) {
		if (x < val) {
			PrintChar('0');
		}
	}

	Serial.print(x,HEX);
}

uint8_t rf12_recvDone () {
#if 0
	if (rxstate != TXRECV) {
		rxfill = 0;
		rf12_len =0;
	}
#endif
    if ((rxstate == TXRECV) && (rxfill > RXFILL_MIN)) {
			if (rf12_len == RXFILL_MIN) {
				Serial.print("I ");
#if 0
				PrintNum((millis() - recvStart) / 1000, ' ', 7);
				Serial.print(" ");
#endif
				PrintNumHex(rf12_status, ' ', 4);
				//recvTimeout = millis() + 1000;
			}
			if (rxfill > rf12_len) { //-- new data
				rf12_len = rxfill;
			    recvTimeout = millis() + 100;
		        //Serial.print('D');
		        //Serial.print(rf12_len, HEX);
				return 0;
			}

			if ((recvTimeout <= millis()) || (rxfill >= RF_MAX) || (millis() > freqTimeout)) {
				rxstate = TXIDLE;
				//rf12_status = rf12_control(0x0000);

			    rf12_xfer(RF_IDLE_MODE);
		        //Serial.print('E');
		        //Serial.print(rf12_len, HEX);
		        //Serial.print('-');
		        //Serial.print(rxfill, HEX);
				//Serial.println();
				rxfill = RXFILL_MIN;
				if (rf12_len > RXFILL_MIN + 2) {
					//rf12_status = 1;
#if 1
					PrintNum((millis() - recvStart) / 1000, ' ', 7);
					Serial.print(" ");
#endif
					return 1;
				} else {
					//Serial.println("rf12_recvStart: 1");
 					rf12_len = RXFILL_MIN;
					rf12_recvStart();
				}
			}
			return 0;
	}
//	Serial.print((millis() - freqTimeout) / 1000, DEC);
 	if (/* (rxstate != TXRECV) && */ (millis() > freqTimeout)) {
		//Serial.println("rf12_recvStart: 2");
		rf12_recvStart();
		return 0;
	}
    if (rxstate == TXIDLE) {
		//Serial.println("rf12_recvStart: 3");
        rf12_recvStart();
   }
    return 0;
}

uint8_t rf12_canSend () {
    // no need to test with interrupts disabled: state TXRECV is only reached
    // outside of ISR and we don't care if rxfill jumps from 0 to 1 here
    if (rxstate == TXRECV && rxfill == RXFILL_MIN &&
            (rf12_byte(0x00) & (RF_RSSI_BIT >> 8)) == 0) {
        rf12_xfer(RF_IDLE_MODE); // stop receiver
        //XXX just in case, don't know whether these RF12 reads are needed!
        // rf12_xfer(0x0000); // status register
        // rf12_xfer(RF_RX_FIFO_READ); // fifo read
        rxstate = TXIDLE;
        rf12_grp = group;
        return 1;
    }
    return 0;
}

void rf12_sendStart (uint8_t hdr) {
    rf12_hdr = hdr & RF12_HDR_DST ? hdr :
                (hdr & ~RF12_HDR_MASK) + (nodeid & NODE_ID);
    if (crypter != 0)
        crypter(1);

    rf12_status = ~0;
#if RF12_VERSION >= 2
    rf12_status = _crc16_update(rf12_status, rf12_grp);
#endif
    rxstate = TXPRE1;
    rf12_xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

void rf12_sendStart (uint8_t hdr, const void* ptr, uint8_t len) {
    rf12_len = len;
    memcpy((void*) rf12_data, ptr, len);
    rf12_sendStart(hdr);
}

// deprecated
void rf12_sendStart (uint8_t hdr, const void* ptr, uint8_t len, uint8_t sync) {
    rf12_sendStart(hdr, ptr, len);
    rf12_sendWait(sync);
}

void rf12_sendWait (uint8_t mode) {
    // wait for packet to actually finish sending
    // go into low power mode, as interrupts are going to come in very soon
    while (rxstate != TXIDLE)
        if (mode) {
            // power down mode is only possible if the fuses are set to start
            // up in 258 clock cycles, i.e. approx 4 us - else must use standby!
            // modes 2 and higher may lose a few clock timer ticks
            set_sleep_mode(mode == 3 ? SLEEP_MODE_PWR_DOWN :
#ifdef SLEEP_MODE_STANDBY
                           mode == 2 ? SLEEP_MODE_STANDBY :
#endif
                                       SLEEP_MODE_IDLE);
            sleep_mode();
        }
}

/*!
  Call this once with the node ID (0-31), frequency band (0-3), and
  optional group (0-255 for RF12B, only 212 allowed for RF12).
*/
void rf12_initialize (uint8_t id, uint8_t band, uint8_t g) {
    nodeid = id;
    group = g;

    spi_initialize();

    pinMode(RFM_IRQ, INPUT);
    digitalWrite(RFM_IRQ, 1); // pull-up

    rf12_xfer(0x0000); // intitial SPI transfer added to avoid power-up problem

    rf12_xfer(RF_SLEEP_MODE); // DC (disable clk pin), enable lbd

    // wait until RFM12B is out of power-up reset, this takes several *seconds*
    rf12_xfer(RF_TXREG_WRITE); // in case we're still in OOK mode
    while (digitalRead(RFM_IRQ) == 0)
        rf12_xfer(0x0000);
#ifndef ITPLUS
    rf12_xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
    rf12_xfer(0xA640); // 868MHz A + 1600
//	rf12_xfer(0xA67C); // IT+ 868.3
    rf12_xfer(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
//    rf12_xfer(0xC614); // IT+ 16.8 Kbps, i.e. 10000/29/(1+20) Kbps

    rf12_xfer(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm
//    rf12_xfer(0x94C2); // VDI,FAST,67kHz,0dBm,-91dBm
    rf12_xfer(0xC2AC); // AL,!ml,DIG,DQD4
//    if (group != 0) {
//        rf12_xfer(0xCA83); // FIFO8,2-SYNC,!ff,DR
//        rf12_xfer(0xCE00 | group); // SYNC=2DXX?
//    } else {
        rf12_xfer(0xCA8B); // FIFO8,1-SYNC,!ff,DR
        rf12_xfer(0xCE2D); // SYNC=2D?
//    }
    rf12_xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN
    rf12_xfer(0x9850); // !mp,90kHz,MAX OUT
    rf12_xfer(0xCC77); // OB1?OB0, LPX,?ddy?DDIT?BW0
    rf12_xfer(0xE000); // NOT USE
    rf12_xfer(0xC800); // NOT USE
    rf12_xfer(0xC049); // 1.66MHz,3.1V
#else // IT+ mods
#ifdef RFM01
#if 0
	//RFM01
		http://fredboboss.free.fr/tx29/tx29_2.php
	0x918A #868, ex, 134khz, clock on
	0xA640 #freq
	0xC080 # clock recovery lock, gain 0, rssi -103
	0xC081
	0xC081
	0xE096 # Wakeup timer
	0xCC0E # low duty cycle POR value
	0xC200 # low battery POR value
	0xC69F # AFC a1a0: 10: drop offset, +15/16, F: alles aan
	0xC46A # data filtes; !AL, Fast, Digital, DQD=2
	0xC813 # data rate POR is 823, 13 hex, is C6 on a RFM12b
	0xCE14 # Output/FIFO mode: FIFO 1, fill on sync, ff, fe: 0: clear

	0x918A #repeat
	0xC69F # repeat AFC
	0xA67C # Change freq!
	0xC080
	0xC081
	0xCE14 # clear FIFO
	0xC080
	0xC081
	0xCE17 # enable fifo
	0xC080
	9 6 4 6 1 4 6 een 1 een
	0xC080
	0xC081
	0xCE14
	0xC080
	0xC081
	0xCE17
	0xC080
	9 6 4 6 1 4 6 een 1 een
#else
	// Try RFM01->RFM012b settings
	rf12_xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
	rf12_xfer(0xA640); // 868MHz A + 1600
   	rf12_xfer(0xC613); // Same as C8 on RFM01
    rf12_xfer(0x94A0); // VDI,FAST,134kHz,0dBm,-91dBm
    rf12_xfer(0xC27A); // Same as C4 on RFM01 but filter is 1 and following bit is 1
#if 0
    rf12_xfer(0xCA8B); // Almost CE on RFM01 FIFO8,1-SYNC,!ff,DR -- 7 no sync...
  	rf12_xfer(0xCED4); // Rkr Try D4? SYNC=2D?
#else
    rf12_xfer(0xCA81); // Almost CE on RFM01 FIFO8,1-SYNC,!ff,DR -- 7 no sync...
  	rf12_xfer(0xCE2D); // Rkr Try D4? SYNC=2D?
#endif
    rf12_xfer(0xC49F); // Is C6 on RFM01
    rf12_xfer(0x9850); // TX register, don't care !mp,90kHz,MAX OUT
    rf12_xfer(0xCC77); // POR OB1?OB0, LPX,?ddy?DDIT?BW0
    rf12_xfer(0xE000); // NOT USE
    rf12_xfer(0xC800); // NOT USE
    rf12_xfer(0xC049); // 1.66MHz,3.1V

    // repeat as in RFM01
	rf12_xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
    rf12_xfer(0xC49F); // Is C6 on RFM01
	rf12_xfer(0xA67C); // 868MHz A + 1600
    rf12_xfer(0xCA81); // Almost CE on RFM01 FIFO8,1-SYNC,!ff,DR -- 7 no sync...
    rf12_xfer(0xCA83); // Almost CE on RFM01 FIFO8,1-SYNC,!ff,DR -- 7 no sync...
#endif
#else // old ITPLUS
    rf12_xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
//    rf12_xfer(0xA640); // 868MHz A + 1600
	//rf12_xfer(0xA67C); // IT+ 868.3, 1660
	rf12_xfer(freq); // + 4 experiment
//	rf12_xfer(0xA708); // IT+ 869
//    rf12_xfer(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
   	rf12_xfer(0xC614); // IT+ 16.8 Kbps, i.e. 10000/29/(1+20) Kbps
//   rf12_xfer(0xC646); // IT+ 4800

    rf12_xfer(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm
 //   rf12_xfer(0x94C2); // VDI,FAST,67kHz,0dBm,-91dBm
    rf12_xfer(0xC2AC); // AL,!ml,DIG,DQD4
#ifdef SOFT_SYNC
    rf12_xfer(0xCA87); // FIFO8,1-SYNC,!ff,DR -- 7 no sync...
#else
    rf12_xfer(0xCA83); // FIFO8,1-SYNC,!ff,DR -- 7 no sync...
#endif
    // try fifo fill always
//    rf12_xfer(0xCA8F); // FIFO8,1-SYNC,!ff,DR
    //rf12_xfer(0xCE2D); // SYNC=2D?
  	rf12_xfer(0xCED4); // Rkr Try D4? SYNC=2D?
//	rf12_xfer(0xCE2D); // SYNC=2D?
    rf12_xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN
    rf12_xfer(0x9850); // !mp,90kHz,MAX OUT
    rf12_xfer(0xCC77); // OB1?OB0, LPX,?ddy?DDIT?BW0
    rf12_xfer(0xE000); // NOT USE
    rf12_xfer(0xC800); // NOT USE
    rf12_xfer(0xC049); // 1.66MHz,3.1V
#endif
#endif

    rxstate = TXIDLE;
//    if ((nodeid & NODE_ID) != 0)
#ifdef SOFT_SYNC
        attachInterrupt(0, rf12_interruptsoftsync, LOW);
#else
        attachInterrupt(0, rf12_interrupt, LOW);
#endif
//    else
//        detachInterrupt(0);
}

void rf12_onOff (uint8_t value) {
    rf12_xfer(value ? RF_XMITTER_ON : RF_IDLE_MODE);
}

uint8_t rf12_config (uint8_t show) {
    uint16_t crc = ~0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i)
        crc = _crc16_update(crc, eeprom_read_byte(RF12_EEPROM_ADDR + i));
    if (crc != 0)
        return 0;

    uint8_t nodeId = 0, group = 0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE - 2; ++i) {
        uint8_t b = eeprom_read_byte(RF12_EEPROM_ADDR + i);
        if (i == 0)
            nodeId = b;
        else if (i == 1)
            group = b;
        else if (b == 0)
            break;
        else if (show)
            Serial.print(b);
    }
    if (show)
        Serial.println();

    rf12_initialize(nodeId, nodeId >> 6, group);
    return nodeId & RF12_HDR_MASK;
}

void rf12_sleep (char n) {
    if (n < 0)
        rf12_control(RF_IDLE_MODE);
    else {
        rf12_control(RF_WAKEUP_TIMER | 0x0500 | n);
        rf12_control(RF_SLEEP_MODE);
        if (n > 0)
            rf12_control(RF_WAKEUP_MODE);
    }
    rxstate = TXIDLE;
}

char rf12_lowbat () {
    return (rf12_control(0x0000) & RF_LBD_BIT) != 0;
}

void rf12_easyInit (uint8_t secs) {
    ezInterval = secs;
}

char rf12_easyPoll () {
    if (rf12_recvDone() && rf12_status == 0) {
        byte myAddr = nodeid & RF12_HDR_MASK;
        if (rf12_hdr == (RF12_HDR_CTL | RF12_HDR_DST | myAddr)) {
            ezPending = 0;
            ezNextSend[0] = 0; // flags succesful packet send
            if (rf12_len > 0)
                return 1;
        }
    }
    if (ezPending > 0) {
        // new data sends should not happen less than ezInterval seconds apart
        // ... whereas retries should not happen less than RETRY_MS apart
        byte newData = ezPending == RETRIES;
        long now = millis();
        if (now >= ezNextSend[newData] && rf12_canSend()) {
            ezNextSend[0] = now + RETRY_MS;
            // must send new data packets at least ezInterval seconds apart
            // ezInterval == 0 is a special case:
            //      for the 868 MHz band: enforce 1% max bandwidth constraint
            //      for other bands: use 100 msec, i.e. max 10 packets/second
            if (newData)
                ezNextSend[1] = now +
                    (ezInterval > 0 ? 1000L * ezInterval
                                    : (nodeid >> 6) == RF12_868MHZ ?
                                            13 * (ezSendLen + 10) : 100);
            rf12_sendStart(RF12_HDR_ACK, ezSendBuf, ezSendLen);
            --ezPending;
        }
    }
    return ezPending ? -1 : 0;
}

char rf12_easySend (const void* data, uint8_t size) {
    if (data != 0 && size != 0) {
        if (ezNextSend[0] == 0 && size == ezSendLen &&
                                    memcmp(ezSendBuf, data, size) == 0)
            return 0;
        memcpy(ezSendBuf, data, size);
        ezSendLen = size;
    }
    ezPending = RETRIES;
    return 1;
}

// XXTEA by David Wheeler, adapted from http://en.wikipedia.org/wiki/XXTEA

#define DELTA 0x9E3779B9
#define MX (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + \
                                            (cryptKey[(uint8_t)((p&3)^e)] ^ z)))

static void cryptFun (uint8_t send) {
    uint32_t y, z, sum, *v = (uint32_t*) rf12_data;
    uint8_t p, e, rounds = 6;

    if (send) {
        // pad with 1..4-byte sequence number
        *(uint32_t*)(rf12_data + rf12_len) = ++seqNum;
        uint8_t pad = 3 - (rf12_len & 3);
        rf12_len += pad;
        rf12_data[rf12_len] &= 0x3F;
        rf12_data[rf12_len] |= pad << 6;
        ++rf12_len;
        // actual encoding
        char n = rf12_len / 4;
        if (n > 1) {
            sum = 0;
            z = v[n-1];
            do {
                sum += DELTA;
                e = (sum >> 2) & 3;
                for (p=0; p<n-1; p++)
                    y = v[p+1], z = v[p] += MX;
                y = v[0];
                z = v[n-1] += MX;
            } while (--rounds);
        }
    } else if (rf12_status == 0) {
        // actual decoding
        char n = rf12_len / 4;
        if (n > 1) {
            sum = rounds*DELTA;
            y = v[0];
            do {
                e = (sum >> 2) & 3;
                for (p=n-1; p>0; p--)
                    z = v[p-1], y = v[p] -= MX;
                z = v[n-1];
                y = v[0] -= MX;
            } while ((sum -= DELTA) != 0);
        }
        // strip sequence number from the end again
        if (n > 0) {
            uint8_t pad = rf12_data[--rf12_len] >> 6;
            rf12_seq = rf12_data[rf12_len] & 0x3F;
            while (pad-- > 0)
                rf12_seq = (rf12_seq << 8) | rf12_data[--rf12_len];
        }
    }
}

void rf12_encrypt (const uint8_t* key) {
    // by using a pointer to cryptFun, we only link it in when actually used
    if (key != 0) {
        for (uint8_t i = 0; i < sizeof cryptKey; ++i)
            ((uint8_t*) cryptKey)[i] = eeprom_read_byte(key + i);
        crypter = cryptFun;
    } else
        crypter = 0;
}
