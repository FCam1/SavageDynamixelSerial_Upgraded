/* Created by FCaminade
 * Date : 16/02/2018
 * Allow the control of dynamixels using Protocol 2.0 like X series 
 * 
 * NB : 
 * Current settings : 
                      - XM_TX_DELAY_TIME = 400us (settings for 1Mbps) for default use (57600bps) this parameter have to be increase ~4000us
 */
#include <time.h>
#include <SavageDynamixelSerial_Upgraded.h>

DynamixelXClass DynamixelX(Serial5); //Allow setup without SoftwareSerial

void setup()
{
  Serial5.begin(1000000); //Allow setup without SoftwareSerial
  DynamixelX.SetDirPin(PG3); //(Baud rate (Default : 57600), Control Pin for Half duplex)

  //Initialization
  DynamixelX.setTorque(0xFE, 0); //(ID, 0 ) Enable EEPROM writing - 0xFE : Broadcast ID
  delay(10);
  DynamixelX.setTorque(0xFE, 1); //(ID, 1) Enable moving
  delay(10);

  Serial.begin(9600); //USB
  while (!Serial)
  {
  }
  delay(400);
  Serial.println("setTorque XM OK");
  Serial.println("Setup OK !");
  Serial.setTimeout(10);
}

float p =0;

void loop()
{
   p = p +0.005;
  float theta = abs(sin(p))*4095;
  DynamixelX.move(1, theta);

  Serial.println(theta);
  //Serial.println(p);

  delay(10);
}
