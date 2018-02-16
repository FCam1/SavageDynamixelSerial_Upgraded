/* Created by FCaminade
 * Date : 16/02/2018
 * Allow the control of dynamixels using Protocol 2.0 like X series 
 * 
 * NB : 
 * Current settings : - Serial4 for protocol 1.0 
                      - serial5 for protocol 2.0 
                      - XM_TX_DELAY_TIME = 400us (settings for 1Mbps) for default use (57600bps) this parameter have to be increase ~4000us
 */

#include <time.h>

#include <SavageDynamixelSerial_Upgraded.h> 

int theta;
float p=1.57;

void setup() {
  DynamixelX.begin(1000000,PG3);//(Baud rate (Default : 57600), Control Pin for Half duplex)
;
  //Initialization
  DynamixelX.setTorque (0xFE,0); //(ID, 0 ) Enable EEPROM writing - 0xFE : Broadcast ID 
  delay (10);   
  DynamixelX.setTorque (0xFE,1); //(ID, 1) Enable moving
  delay (1);
  
  Serial.begin (9600); //USB
  while (!Serial) {}
  delay(400);
  Serial.println("setTorque XM OK");
  Serial.println("Setup OK !");
  Serial.setTimeout(10);

}

void loop() {

  p=p+0.01;
  if (p>1.57) p-0.01;
  
  
  theta = labs(255*cos(p));
  Serial.println(theta);
    
  DynamixelX.move(1,theta ); 
  
  delay (10);

}
