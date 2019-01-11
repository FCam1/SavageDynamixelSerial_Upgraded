/*
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 Created by Savage on 27/01/11.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *****************************************************************************
 Modifications:
 
 25/07/2011 - Eliminado la modificacion serial para ser modificada dentro del mismo Hardware Serial.
 25/07/2011 - Modificado la funcion setBD() para aceptar todas la velocidades sin PDF.
 25/07/2011 - Agregada la funcion de Rotacion Continua.
 26/07/2011 - Agregada la funcion begin sin seteo de Direction_Pin.
 25/07/2011 - Agregada la funcion Reset.
 26/07/2011 - Agregada la funcion Reg_Write en move y moveSpeed.
 26/07/2011 - Agregada la funcion Action.
 13/12/2011 - Arreglado el manejo y envio de variables.
 22/12/2011 - Compatible con la actualizacion Arduino 1.0.
 10/01/2012 - Utilizacion de Macros y eliminacion codigo no necesario.
 11/01/2012 - Agregadas las funciones:
              int setTempLimit(unsigned char ID, unsigned char Temperature);
              int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
              int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
              int setMaxTorque(unsigned char ID, int MaxTorque);
			  int setSRL(unsigned char ID, unsigned char SRL);
              int setRDT(unsigned char ID, unsigned char RDT);
              int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
			  int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
              int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
              int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
 15/01/2012 - Agregadas las funciones:
              int setPunch(unsigned char ID, int Punch);
              int moving(unsigned char ID);
              int lockRegister(unsigned char ID);
              int RWStatus(unsigned char ID);
              int readSpeed(unsigned char ID);
              int readLoad(unsigned char ID); 
 
 TODO:
 
 FUNCION SYNCWRITE.
 
 *****************************************************************************
 
 Contact: savageelectronics@gmail.com 
 Web:     http://savageelectrtonics.blogspot.com/
 Autor:   Josue Alejandro Savage
 *****************************************************************************
  Modified by FCaminade 
 * 14/10/2017 :Extension for protocol 2.0. Add of DynamixelXClass
 
 
 */

#ifndef DynamixelSerial_h
#define DynamixelSerial_h
//////////////////////////PROTOCOL 1.0//////////////////////////////////////
// EEPROM AREA  ///////////////////////////////////////////////////////////
#define AX_MODEL_NUMBER_L 0
#define AX_MODEL_NUMBER_H 1
#define AX_VERSION 2
#define AX_ID 3
#define AX_BAUD_RATE 4
#define AX_RETURN_DELAY_TIME 5
#define AX_CW_ANGLE_LIMIT_L 6
#define AX_CW_ANGLE_LIMIT_H 7
#define AX_CCW_ANGLE_LIMIT_L 8
#define AX_CCW_ANGLE_LIMIT_H 9
#define AX_SYSTEM_DATA2 10
#define AX_LIMIT_TEMPERATURE 11
#define AX_DOWN_LIMIT_VOLTAGE 12
#define AX_UP_LIMIT_VOLTAGE 13
#define AX_MAX_TORQUE_L 14
#define AX_MAX_TORQUE_H 15
#define AX_RETURN_LEVEL 16
#define AX_ALARM_LED 17
#define AX_ALARM_SHUTDOWN 18
#define AX_OPERATING_MODE 19
#define AX_DOWN_CALIBRATION_L 20
#define AX_DOWN_CALIBRATION_H 21
#define AX_UP_CALIBRATION_L 22
#define AX_UP_CALIBRATION_H 23

// RAM AREA  //////////////////////////////////////////////////////////////
#define AX_TORQUE_ENABLE 24
#define AX_LED 25
#define AX_CW_COMPLIANCE_MARGIN 26
#define AX_CCW_COMPLIANCE_MARGIN 27
#define AX_CW_COMPLIANCE_SLOPE 28
#define AX_CCW_COMPLIANCE_SLOPE 29
#define AX_GOAL_POSITION_L 30
#define AX_GOAL_POSITION_H 31
#define AX_GOAL_SPEED_L 32
#define AX_GOAL_SPEED_H 33
#define AX_TORQUE_LIMIT_L 34
#define AX_TORQUE_LIMIT_H 35
#define AX_PRESENT_POSITION_L 36
#define AX_PRESENT_POSITION_H 37
#define AX_PRESENT_SPEED_L 38
#define AX_PRESENT_SPEED_H 39
#define AX_PRESENT_LOAD_L 40
#define AX_PRESENT_LOAD_H 41
#define AX_PRESENT_VOLTAGE 42
#define AX_PRESENT_TEMPERATURE 43
#define AX_REGISTERED_INSTRUCTION 44
#define AX_PAUSE_TIME 45
#define AX_MOVING 46
#define AX_LOCK 47
#define AX_PUNCH_L 48
#define AX_PUNCH_H 49

// Status Return Levels ///////////////////////////////////////////////////////////////
#define AX_RETURN_NONE 0
#define AX_RETURN_READ 1
#define AX_RETURN_ALL 2

// Instruction Set ///////////////////////////////////////////////////////////////
#define AX_PING 1
#define AX_READ_DATA 2
#define AX_WRITE_DATA 3
#define AX_REG_WRITE 4
#define AX_ACTION 5
#define AX_RESET 6
#define AX_SYNC_WRITE 131

// Specials ///////////////////////////////////////////////////////////////
#define OFF 0
#define ON 1
#define LEFT 0
#define RIGTH 1
#define AX_BYTE_READ 1
#define AX_BYTE_READ_POS 2
#define AX_RESET_LENGTH 2
#define AX_ACTION_LENGTH 2
#define AX_ID_LENGTH 4
#define AX_LR_LENGTH 4
#define AX_SRL_LENGTH 4
#define AX_RDT_LENGTH 4
#define AX_LEDALARM_LENGTH 4
#define AX_SALARM_LENGTH 4
#define AX_TL_LENGTH 4
#define AX_VL_LENGTH 6
#define AX_CM_LENGTH 6
#define AX_CS_LENGTH 6
#define AX_CCW_CW_LENGTH 8
#define AX_BD_LENGTH 4
#define AX_TEM_LENGTH 4
#define AX_MOVING_LENGTH 4
#define AX_RWS_LENGTH 4
#define AX_VOLT_LENGTH 4
#define AX_LED_LENGTH 4
#define AX_TORQUE_LENGTH 4
#define AX_POS_LENGTH 4
#define AX_GOAL_LENGTH 5
#define AX_MT_LENGTH 5
#define AX_PUNCH_LENGTH 5
#define AX_SPEED_LENGTH 5
#define AX_GOAL_SP_LENGTH 7
#define AX_ACTION_CHECKSUM 250
#define BROADCAST_ID 254
#define AX_START 255
#define AX_CCW_AL_L 255
#define AX_CCW_AL_H 3
#define TIME_OUT 200    //500000         // Este parametro depende de la velocidad de transmision
#define TX_DELAY_TIME 1 //1us     // Este parametro depende de la velocidad de transmision - pero pueden ser cambiados para mayor velocidad.
#define Tx_MODE 1
#define Rx_MODE 0
#define LOCK 1

//////////////////////////PROTOCOL 2.0//////////////////////////////////////
// EEPROM AREA  ///////////////////////////////////////////////////////////
#define XM_BAUD_RATE 8
#define XM_LED 65

// RAM AREA  //////////////////////////////////////////////////////////////
#define XM_RETURN_DELAY_TIME 9
#define XM_TORQUE_ENABLE 64
#define XM_PROFILE_ACC 108
#define XM_PROFILE_VEL 112
#define XM_GOAL_POSITION 116
#define XM_PRESENT_CURRENT 126
#define XM_PRESENT_POSITION 132

// Status Return Levels ///////////////////////////////////////////////////////////////

// Instruction Set ///////////////////////////////////////////////////////////////
#define XM_PING 1
#define XM_READ 2
#define XM_WRITE 3
#define XM_SYNC_WRITE 131
#define XM_SYNC_READ 130
// Specials ///////////////////////////////////////////////////////////////
#define XM_START_FF 255
#define XM_START_FD 253
#define XM_0 0
#define XM_RDT_LENGTH 6
#define XM_BD_LENGTH 6
#define XM_TORQUE_LENGTH 6
#define XM_GOAL_LENGTH 9
#define XM_SYNC_WRITE_LENGTH 4
#define XM_TX_DELAY_TIME 1

#define LENGTH_2 2
#define LENGTH_3 3
#define LENGTH_4 4
#define LENGTH_6 6 //number of parameters+3
#define LENGTH_7 7
#define LENGTH_9 9
#define LENGTH_17 17

#include <inttypes.h>

class DynamixelClass
{
private:
  unsigned char Checksum;
  unsigned char Direction_Pin;
  unsigned int Time_Counter;
  unsigned char Incoming_Byte;
  unsigned char Position_High_Byte;
  unsigned char Position_Low_Byte;
  unsigned char Speed_High_Byte;
  unsigned char Speed_Low_Byte;
  unsigned char Load_High_Byte;
  unsigned char Load_Low_Byte;

  int Moving_Byte;
  int RWS_Byte;
  int Speed_Long_Byte;
  int Load_Long_Byte;
  int Position_Long_Byte;
  int Temperature_Byte;
  int Voltage_Byte;
  int Error_Byte;

  int read_error(void);
  Stream &serial_;

public:
  DynamixelClass(Stream &serial);
  void SetDirPin(unsigned char directionPin);
  void begin(long baud, unsigned char directionPin); //depreciated
  void begin(long baud);                             //depreciated
  void end(void);

  int reset(unsigned char ID);
  int ping(unsigned char ID);

  int setID(unsigned char ID, unsigned char newID);
  int setBD(unsigned char ID, long baud);

  int move(unsigned char ID, int Position);
  int moveSpeed(unsigned char ID, int Position, int Speed);
  int setEndless(unsigned char ID, bool Status);
  int turn(unsigned char ID, bool SIDE, int Speed);
  int moveRW(unsigned char ID, int Position);
  int moveSpeedRW(unsigned char ID, int Position, int Speed);

  void action(void);

  int setTempLimit(unsigned char ID, unsigned char Temperature);
  int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
  int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
  int setMaxTorque(unsigned char ID, int MaxTorque);
  int setSRL(unsigned char ID, unsigned char SRL);
  int setRDT(unsigned char ID, unsigned char RDT);
  int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
  int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
  int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
  int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
  int setPunch(unsigned char ID, int Punch);

  int moving(unsigned char ID);
  int lockRegister(unsigned char ID);
  int RWStatus(unsigned char ID);

  int readTemperature(unsigned char ID);
  int readVoltage(unsigned char ID);
  int readPosition(unsigned char ID);
  int readPosition(unsigned char ID, int *Pos_Long_Byte);
  int readSpeed(unsigned char ID);
  int readLoad(unsigned char ID);

  int torqueStatus(unsigned char ID, bool Status);
  int ledStatus(unsigned char ID, bool Status);

  int synWritePos(unsigned char ID1, int Position1, unsigned char ID2, int Position2);
};

class DynamixelXClass
{
private:
  unsigned char Direction_Pin;
  unsigned int Time_Counter;
  unsigned char Incoming_Byte;
  unsigned char Position_High_Byte;
  unsigned char Position_Low_Byte;
  unsigned char Speed_High_Byte;
  unsigned char Speed_Low_Byte;
  unsigned char Load_High_Byte;
  unsigned char Load_Low_Byte;
  unsigned char detectID;
  unsigned char Model_Low_Byte;
  unsigned char Model_High_Byte;
  unsigned char Fw_ver;

  int Moving_Byte;
  int RWS_Byte;
  int Speed_Long_Byte;
  int Load_Long_Byte;
  int Position_Long_Byte;
  int Temperature_Byte;
  int Voltage_Byte;
  int Error_Byte;
  int Model_Long_Byte;

  int read_error(void);
  unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
  Stream &serialx_;

public:
  DynamixelXClass(Stream &serialx);
  void SetDirPin(unsigned char directionPin);
  void begin(long baud, unsigned char directionPin); //depreciated
  int move(unsigned char ID, int Position);
  int setRDT(unsigned char ID, unsigned char RDT); // SET return delay time
  int ledStatus(unsigned char ID, bool Status);
  int setBD(unsigned char ID, int baud); //0:9600, 1:57600(default), 2:115200, 3:1M, 4:2M, 5:3M, 6:4M, 7:4.5M
  int setTorque(unsigned char ID, bool torque);
  int synWritePos(unsigned char ID1, int Position1, unsigned char ID2, int Position2);
  int setProfileAcc(unsigned char ID, int acc_range);
  int setProfileVel(unsigned char ID, int vel_range);

  int readPosition(unsigned char ID);
  int readPosition(unsigned char ID, int *Pos_Long_Byte);
  struct ReturnPacket2 ping(unsigned char ID);
  int syncReadPos(unsigned char ID1, int *Pos_Long_Byte1, unsigned char ID2, int *Pos_Long_Byte2); //Sync Read of the position of 2 motors
  int syncReadCur(unsigned char ID1, unsigned char ID2);                                           //Sync Read of the current of 2 motors
};

extern DynamixelClass Dynamixel;
extern DynamixelXClass DynamixelX;

//Structures
struct ReturnPacket2
{
  uint16_t value1;
  uint16_t value2;
};

struct ReturnPacket3
{
  uint16_t value1;
  uint16_t value2;
  uint16_t value3;
};

struct structest
{
  uint16_t a;
  uint16_t b;
  uint16_t c;
};

struct PacketSize0
{                                                                    //All data Packets with 3 parameters
  unsigned char header[3] = {XM_START_FF, XM_START_FF, XM_START_FD}; //defined
  unsigned char reserved = XM_0;                                     //defined
  unsigned char id;                                                  //parameter
  unsigned char packetlenght_L = LENGTH_3;                           //defined
  unsigned char packetlenght_H = XM_0;                               //defined
  unsigned char instruction;                                         //parameter
  unsigned char crc_L;                                               //parameter
  unsigned char crc_H;                                               //parameter
};
struct PacketSize3
{                                                                    //All data Packets with 3 parameters
  unsigned char header[3] = {XM_START_FF, XM_START_FF, XM_START_FD}; //defined
  unsigned char reserved = XM_0;                                     //defined
  unsigned char id;                                                  //parameter
  unsigned char packetlenght_L = LENGTH_6;                           //defined
  unsigned char packetlenght_H = XM_0;                               //defined
  unsigned char instruction;                                         //parameter
  unsigned char param1;                                              //adress Low
  unsigned char param2 = XM_0;                                       //adress High
  unsigned char param3;                                              //value
  unsigned char crc_L;                                               //parameter
  unsigned char crc_H;                                               //parameter
};

struct PacketSize4
{                                                                    //All data Packets with 4 parameters
  unsigned char header[3] = {XM_START_FF, XM_START_FF, XM_START_FD}; //defined
  unsigned char reserved = XM_0;                                     //defined
  unsigned char id;                                                  //parameter
  unsigned char packetlenght_L = LENGTH_7;                           //defined
  unsigned char packetlenght_H = XM_0;                               //defined
  unsigned char instruction;                                         //parameter
  unsigned char param1;                                              //adress Low
  unsigned char param2 = XM_0;                                       //adress High
  unsigned char param3;                                              //value Low
  unsigned char param4;                                              //value High
  unsigned char crc_L;                                               //parameter
  unsigned char crc_H;                                               //parameter
};

struct PacketSize6
{                                                                    //All data Packets with 6 parameters
  unsigned char header[3] = {XM_START_FF, XM_START_FF, XM_START_FD}; //defined
  unsigned char reserved = XM_0;                                     //defined
  unsigned char id;                                                  //parameter
  unsigned char packetlenght_L = LENGTH_9;                           //defined
  unsigned char packetlenght_H = XM_0;                               //defined
  unsigned char instruction;                                         //parameter
  unsigned char param1;                                              //adress Low
  unsigned char param2 = XM_0;                                       //adress High
  unsigned char param3;                                              //value Low
  unsigned char param4;                                              //value High
  unsigned char param5;                                              //value
  unsigned char param6;                                              //value
  unsigned char crc_L;                                               //parameter
  unsigned char crc_H;                                               //parameter
};

struct PacketSync14
{                                                                    //Sync for 2 motors and 14 parameters
  unsigned char header[3] = {XM_START_FF, XM_START_FF, XM_START_FD}; //defined
  unsigned char reserved = XM_0;                                     //defined
  unsigned char id = BROADCAST_ID;
  unsigned char packetlenght_L = LENGTH_17;  //defined
  unsigned char packetlenght_H = XM_0;       //defined
  unsigned char instruction = XM_SYNC_WRITE; //defined
  unsigned char param1;                      //adress Low
  unsigned char param2 = XM_0;               //adress High
  unsigned char param3;                      //datalength_L
  unsigned char param4;                      //datalength_H
  unsigned char param5;                      //Parameter 5
  unsigned char param6;                      //Parameter 6
  unsigned char param7;                      //Parameter 7
  unsigned char param8;                      //Parameter 8
  unsigned char param9;                      //Parameter 9
  unsigned char param10;                     //Parameter 10
  unsigned char param11;                     //Parameter 11
  unsigned char param12;                     //Parameter 12
  unsigned char param13;                     //Parameter 13
  unsigned char param14;                     //Parameter 14
  unsigned char crc_L;                       //parameter
  unsigned char crc_H;                       //parameter
};

#endif
