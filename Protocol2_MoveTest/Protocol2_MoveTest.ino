/* Created by FCaminade
 * Date : 16/02/2018
 * Allow the control of dynamixels using Protocol 2.0 like X series 
 */

#include <time.h>

#include <SavageDynamixelSerial_Upgraded.h> ///ATENTION SERIAL 3 || Tx, Rx pins have to be modified in DynamixelSerial_Modified.cpp ||

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
    
  DynamixelX.move(1,theta ); //DYNAMIXEL AX Right
  
  delay (10);

}
