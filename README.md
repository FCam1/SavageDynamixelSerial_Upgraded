# SavageDynamixelSerial_Upgraded-

Based on the famous "Savage librarie for dynamixel AX" http://savageelectronics.blogspot.fr/2011/01/arduino-y-dynamixel-ax-12.html 

The original librarie allow to control only the dynamixels with Protocol 1.0. This new librarie have been extended to control the Dynamixels with Protocol 2.0. 

Current State of librarie: 
begin(long baud, unsigned char directionPin)
setRDT(unsigned char ID, unsigned char RDT)
setBD(unsigned char ID, int baud)
setTorque(unsigned char ID, bool torque);
synWritePos(unsigned char ID1, int Position1,unsigned char ID2, int Position2)


Current settings : 
- Serial4 for protocol 1.0 
- serial5 for protocol 2.0 
- XM_TX_DELAY_TIME = 400us (settings for 1Mbps) for default use (57600bps) this parameter have to be increase ~4000us


Installation : 
1) Download .zip
2) In Arduino IDE : Croquis>Inclure une bibliothèque>Ajouter la bibliothèque .zip
