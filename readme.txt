>> Als Parameter fuer den RFM12B benutze ich: 
>> Baudrate:            16800 
>> Frequenz:            868300 
>> Modulation:          FSK 
>> TX-Frequenzhub:      45KHz 
>> Empfangerbandbreite: 67KHz 
>> Die sync-bytes vom RFM12B sind 0x2dd4 mit einer 16bit Preambel 
>> (1010101...). Ein Paket vom TX21IT+ sieht dann so aus (abzueglich 
>> Preambel & Syncbyte: 0x97 0xa4 0x86 0x21 0xa8 

4. Frequency Setting Command
f11..f0: Set operation frequency:
868band: Fc=860+F*0.0050 MHz

5. Data Rate Command
r6..r0: Set data rate: 
BR=10000000/29/(R+1)/(1+cs*7)   

6. Receiver Control Command
1 1 0 67

9. Synchron pattern Command 
bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR 
 1 1 0 0 1 1 1 0 b7 b6 b5 b4 b3 b2 b1 b0 CED4h 
This command is used to reprogram the synchronic pattern;

12. TX Configuration Control Command 
bit 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 POR 
 1 0 0 1 1 0 0 mp m3 m2 m1 m0 0  p2 p1 p0  9800h 
m: select modulation polarity 
m2..m0: select frequency deviation:
0 0 1 0 45


10101010 = AA
2D D4

0  0  0  0 81  5 D4 47  9  C  0  0 91 E0  0  0 F6 E5  0  0 68  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0