/*
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 Created by Savage on 27/01/11.
 Autor:   Josue Alejandro Savage
 
 Modified by FCaminade 
 * 14/10/2017 :Extension for protocol 2.0. Add of DynamixelXClass
 
 */

#if defined(ARDUINO) && ARDUINO >= 100 // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SavageDynamixelSerial_Upgraded.h"

// Macro for the selection of the Serial Port
//Protocol 1.0
#define sendData(args) (serial_.write(args))  // Write Over Serial
#define availableData() (serial_.available()) // Check Serial Data Available
#define readData() (serial_.read())           // Read Serial Data
#define peekData() (serial_.peek())           // Peek Serial Data
//#define beginCom(args) (serial_.begin(args))  // Begin Serial Comunication
//#define endCom() (serial_.end())              // End Serial Comunication
#define axflush() (serial_.flush()) // Wait the end of the transfert
//Protocol 2.0
#define sendDataXm(args) (serialx_.write(args))        // Write Over Serial
#define sendDataX(args, gg) (serialx_.write(args, gg)) // Write Over Serial
#define availableDataXm() (serialx_.available())       // Check Serial Data Available
#define readDataXm() (serialx_.read())                 // Read Serial Data
#define peekDataXm() (serialx_.peek())                 // Peek Serial Data
//#define beginComXm(args) (serialx_.begin(args))        // Begin Serial Comunication
//#define endComXm() (serialx_.end())                    // End Serial Comunication
#define xmflush() (serialx_.flush()) // Wait the end of the transfert
// Macro for Timing

#define delayus(args) (delayMicroseconds(args)) // Delay Microseconds

// Macro for Comunication Flow Control

#define setDPin(DirPin, Mode) (pinMode(DirPin, Mode))        // Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin, Mode) (digitalWrite(DirPin, Mode)) // Switch to TX/RX Mode

// Private Methods //////////////////////////////////////////////////////////////

int DynamixelClass::read_error(void)
{
  Time_Counter = 0;
  while ((availableData() < 5) & (Time_Counter < TIME_OUT))
  { // Wait for Data
    Time_Counter++;
    delayus(1);
  }

  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();              // Start Bytes
      readData();              // Ax-12 ID
      readData();              // Length
      Error_Byte = readData(); // Error
      return (Error_Byte);
    }
  }
  return (-1); // No Ax Response
}

int DynamixelXClass::read_error(void)
{
  Time_Counter = 0;
  while ((availableDataXm() < 9) & (Time_Counter < TIME_OUT))
  { // Wait the end of the return packet
    Time_Counter++;
    delayus(1);
  }

  while (availableDataXm() > 0)
  {
    Incoming_Byte = readDataXm(); //FF
    if ((Incoming_Byte == 255) & (peekDataXm() == 255))
    {
      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FD
      readDataXm();              // reserved
      readDataXm();              //  ID
      readDataXm();              // Length L
      readDataXm();              // Length H
      readDataXm();              // Instruction
      Error_Byte = readDataXm(); // Error
      return (Error_Byte);
    }
  }
  return (-1); // No Ax Response
}

//ROBOTIS CRC16 CALCULATOR
//crc_accum : set as 0
//*data_blk_ptr : packet to send
//data_blk_size : number of bytes in the packet excluding the CRC
//Return 16 bit CRC value

unsigned short DynamixelXClass::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{

  unsigned short i, j;

  unsigned short crc_table[256] = {

      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,

      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,

      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,

      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,

      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,

      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,

      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,

      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,

      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,

      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,

      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,

      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,

      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,

      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,

      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,

      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,

      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,

      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,

      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,

      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,

      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,

      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,

      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,

      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,

      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,

      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,

      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,

      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,

      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,

      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,

      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,

      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202

  };
  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;

    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}
//Privates structures
struct ReturnPacket2 returnpacket2;
struct ReturnPacket2 *ptr_returnpacket2;

struct ReturnPacket3 returnpacket3;
struct ReturnPacket3 *ptr_returnpacket3;

struct PacketSize0 packetsize0;
struct PacketSize0 *ptr_packetsize0;

struct PacketSize3 packetsize3;
struct PacketSize3 *ptr_packetsize3;

struct PacketSize6 packetsize6;
struct PacketSize6 *ptr_packetsize6;

struct PacketSize4 packetsize4;
struct PacketSize4 *ptr_packetsize4;

struct PacketSync14 packetsync14;
struct PacketSync14 *ptr_packetsync14;

// Public Methods //////////////////////////////////////////////////////////////
//Protocol 1.0
DynamixelClass::DynamixelClass(Stream &serial)
    : serial_(serial) {}

void DynamixelClass::SetDirPin(unsigned char directionPin)
{
  Direction_Pin = directionPin;
  setDPin(Direction_Pin, OUTPUT);
}

void DynamixelClass::begin(long baud, unsigned char directionPin)
{
  Direction_Pin = directionPin;
  setDPin(Direction_Pin, OUTPUT);
  //beginCom(baud);
}

void DynamixelClass::begin(long baud)
{
  //beginCom(baud);
}

void DynamixelClass::end()
{
  //endCom();
}

int DynamixelClass::reset(unsigned char ID)
{
  Checksum = (~(ID + AX_RESET_LENGTH + AX_RESET)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_RESET_LENGTH);
  sendData(AX_RESET);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error());
}

int DynamixelClass::ping(unsigned char ID)
{
  Checksum = (~(ID + AX_READ_DATA + AX_PING)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_READ_DATA);
  sendData(AX_PING);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error());
}

int DynamixelClass::setID(unsigned char ID, unsigned char newID)
{
  Checksum = (~(ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + newID)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_ID_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_ID);
  sendData(newID);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::setBD(unsigned char ID, long baud)
{
  unsigned char Baud_Rate = (2000000 / baud) - 1;
  Checksum = (~(ID + AX_BD_LENGTH + AX_WRITE_DATA + AX_BAUD_RATE + Baud_Rate)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_BD_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_BAUD_RATE);
  sendData(Baud_Rate);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::move(unsigned char ID, int Position)
{
  char Position_H, Position_L;
  Position_H = Position >> 8; // 16 bits - 2 x 8 bits variables
  Position_L = Position;
  Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_GOAL_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_GOAL_POSITION_L);
  sendData(Position_L);
  sendData(Position_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::moveSpeed(unsigned char ID, int Position, int Speed)
{
  char Position_H, Position_L, Speed_H, Speed_L;
  Position_H = Position >> 8;
  Position_L = Position; // 16 bits - 2 x 8 bits variables
  Speed_H = Speed >> 8;
  Speed_L = Speed; // 16 bits - 2 x 8 bits variables
  Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_GOAL_SP_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_GOAL_POSITION_L);
  sendData(Position_L);
  sendData(Position_H);
  sendData(Speed_L);
  sendData(Speed_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::setEndless(unsigned char ID, bool Status)
{
  if (Status)
  {
    char AX_CCW_AL_LT = 0; // Changing the CCW Angle Limits for Full Rotation.
    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L)) & 0xFF;

    switchCom(Direction_Pin, Tx_MODE);
    sendData(AX_START); // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_CCW_ANGLE_LIMIT_L);
    sendData(AX_CCW_AL_LT);
    sendData(AX_CCW_AL_LT);
    sendData(Checksum);
    axflush();
    delayus(TX_DELAY_TIME);
    switchCom(Direction_Pin, Rx_MODE);

    return (read_error());
  }
  else
  {
    turn(ID, 0, 0);
    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H)) & 0xFF;

    switchCom(Direction_Pin, Tx_MODE);
    sendData(AX_START); // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_CCW_ANGLE_LIMIT_L);
    sendData(AX_CCW_AL_L);
    sendData(AX_CCW_AL_H);
    sendData(Checksum);
    axflush();
    delayus(TX_DELAY_TIME);
    switchCom(Direction_Pin, Rx_MODE);

    return (read_error()); // Return the read error
  }
}

int DynamixelClass::turn(unsigned char ID, bool SIDE, int Speed)
{
  if (SIDE == 0)
  { // Move Left///////////////////////////

    char Speed_H, Speed_L;
    Speed_H = Speed >> 8;
    Speed_L = Speed; // 16 bits - 2 x 8 bits variables
    Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

    switchCom(Direction_Pin, Tx_MODE);
    sendData(AX_START); // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_SPEED_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_GOAL_SPEED_L);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
    axflush();
    delayus(TX_DELAY_TIME);
    switchCom(Direction_Pin, Rx_MODE);

    return (read_error()); // Return the read error
  }
  else
  { // Move Rigth////////////////////
    char Speed_H, Speed_L;
    Speed_H = (Speed >> 8) + 4;
    Speed_L = Speed; // 16 bits - 2 x 8 bits variables
    Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

    switchCom(Direction_Pin, Tx_MODE);
    sendData(AX_START); // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_SPEED_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_GOAL_SPEED_L);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
    axflush();
    delayus(TX_DELAY_TIME);
    switchCom(Direction_Pin, Rx_MODE);

    return (read_error()); // Return the read error
  }
}

int DynamixelClass::moveRW(unsigned char ID, int Position)
{
  char Position_H, Position_L;
  Position_H = Position >> 8; // 16 bits - 2 x 8 bits variables
  Position_L = Position;
  Checksum = (~(ID + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_GOAL_LENGTH);
  sendData(AX_REG_WRITE);
  sendData(AX_GOAL_POSITION_L);
  sendData(Position_L);
  sendData(Position_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::moveSpeedRW(unsigned char ID, int Position, int Speed)
{
  char Position_H, Position_L, Speed_H, Speed_L;
  Position_H = Position >> 8;
  Position_L = Position; // 16 bits - 2 x 8 bits variables
  Speed_H = Speed >> 8;
  Speed_L = Speed; // 16 bits - 2 x 8 bits variables
  Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_GOAL_SP_LENGTH);
  sendData(AX_REG_WRITE);
  sendData(AX_GOAL_POSITION_L);
  sendData(Position_L);
  sendData(Position_H);
  sendData(Speed_L);
  sendData(Speed_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

void DynamixelClass::action()
{
  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(BROADCAST_ID);
  sendData(AX_ACTION_LENGTH);
  sendData(AX_ACTION);
  sendData(AX_ACTION_CHECKSUM);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);
}

int DynamixelClass::torqueStatus(unsigned char ID, bool Status)
{
  Checksum = (~(ID + AX_TORQUE_LENGTH + AX_WRITE_DATA + AX_TORQUE_ENABLE + Status)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_TORQUE_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_TORQUE_ENABLE);
  sendData(Status);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::ledStatus(unsigned char ID, bool Status)
{
  Checksum = (~(ID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + Status)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_LED_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_LED);
  sendData(Status);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::readTemperature(unsigned char ID)
{
  Checksum = (~(ID + AX_TEM_LENGTH + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_TEM_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_PRESENT_TEMPERATURE);
  sendData(AX_BYTE_READ);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  Temperature_Byte = -1;
  Time_Counter = 0;
  while ((availableData() < 6) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }

  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
        return (Error_Byte * (-1));
      Temperature_Byte = readData(); // Temperature
    }
  }
  return (Temperature_Byte); // Returns the read temperature
}

int DynamixelClass::readPosition(unsigned char ID)
{
  Checksum = (~(ID + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_POS_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_PRESENT_POSITION_L);
  sendData(AX_BYTE_READ_POS);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  Position_Long_Byte = -1;
  Time_Counter = 0;

  while ((availableData() < 7) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }
  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
        return (Error_Byte * (-1));

      Position_Low_Byte = readData(); // Position Bytes
      Position_High_Byte = readData();
      Position_Long_Byte = Position_High_Byte << 8;
      Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
    }
  }
  return (Position_Long_Byte); // Returns the read position
}

int DynamixelClass::readPosition(unsigned char ID, int *Pos_Long_Byte)
{
  Checksum = (~(ID + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_POS_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_PRESENT_POSITION_L);
  sendData(AX_BYTE_READ_POS);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  *Pos_Long_Byte = -1;
  Time_Counter = 0;

  while ((availableData() < 7) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }
  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
      {
        return (Error_Byte);
      }
      Position_Low_Byte = readData(); // Position Bytes
      Position_High_Byte = readData();
      *Pos_Long_Byte = Position_High_Byte << 8 | Position_Low_Byte;
      //readData(); //Checksum
    }
  }
}

int DynamixelClass::readVoltage(unsigned char ID)
{
  Checksum = (~(ID + AX_VOLT_LENGTH + AX_READ_DATA + AX_PRESENT_VOLTAGE + AX_BYTE_READ)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_VOLT_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_PRESENT_VOLTAGE);
  sendData(AX_BYTE_READ);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  Voltage_Byte = -1;
  Time_Counter = 0;
  while ((availableData() < 6) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }

  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
        return (Error_Byte * (-1));
      Voltage_Byte = readData(); // Voltage
    }
  }
  return (Voltage_Byte); // Returns the read Voltage
}

int DynamixelClass::setTempLimit(unsigned char ID, unsigned char Temperature)
{
  Checksum = (~(ID + AX_TL_LENGTH + AX_WRITE_DATA + AX_LIMIT_TEMPERATURE + Temperature)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_TL_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_LIMIT_TEMPERATURE);
  sendData(Temperature);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error());
}

int DynamixelClass::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage)
{
  Checksum = (~(ID + AX_VL_LENGTH + AX_WRITE_DATA + AX_DOWN_LIMIT_VOLTAGE + DVoltage + UVoltage)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_VL_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_DOWN_LIMIT_VOLTAGE);
  sendData(DVoltage);
  sendData(UVoltage);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error());
}

int DynamixelClass::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit)
{
  char CW_H, CW_L, CCW_H, CCW_L;
  CW_H = CWLimit >> 8;
  CW_L = CWLimit; // 16 bits - 2 x 8 bits variables
  CCW_H = CCWLimit >> 8;
  CCW_L = CCWLimit;
  Checksum = (~(ID + AX_VL_LENGTH + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L + CW_H + CW_L + AX_CCW_ANGLE_LIMIT_L + CCW_H + CCW_L)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_CCW_CW_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_CW_ANGLE_LIMIT_L);
  sendData(CW_L);
  sendData(CW_H);
  sendData(AX_CCW_ANGLE_LIMIT_L);
  sendData(CCW_L);
  sendData(CCW_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error());
}

int DynamixelClass::setMaxTorque(unsigned char ID, int MaxTorque)
{
  char MaxTorque_H, MaxTorque_L;
  MaxTorque_H = MaxTorque >> 8; // 16 bits - 2 x 8 bits variables
  MaxTorque_L = MaxTorque;
  Checksum = (~(ID + AX_MT_LENGTH + AX_WRITE_DATA + AX_MAX_TORQUE_L + MaxTorque_L + MaxTorque_H)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_MT_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_MAX_TORQUE_L);
  sendData(MaxTorque_L);
  sendData(MaxTorque_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::setSRL(unsigned char ID, unsigned char SRL)
{
  Checksum = (~(ID + AX_SRL_LENGTH + AX_WRITE_DATA + AX_RETURN_LEVEL + SRL)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_SRL_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_RETURN_LEVEL);
  sendData(SRL);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::setRDT(unsigned char ID, unsigned char RDT)
{
  Checksum = (~(ID + AX_RDT_LENGTH + AX_WRITE_DATA + AX_RETURN_DELAY_TIME + RDT)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_RDT_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_RETURN_DELAY_TIME);
  sendData(RDT);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm)
{
  Checksum = (~(ID + AX_LEDALARM_LENGTH + AX_WRITE_DATA + AX_ALARM_LED + LEDAlarm)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_LEDALARM_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_ALARM_LED);
  sendData(LEDAlarm);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::setShutdownAlarm(unsigned char ID, unsigned char SALARM)
{
  Checksum = (~(ID + AX_SALARM_LENGTH + AX_ALARM_SHUTDOWN + AX_ALARM_LED + SALARM)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_SALARM_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_ALARM_SHUTDOWN);
  sendData(SALARM);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin)
{
  Checksum = (~(ID + AX_CM_LENGTH + AX_WRITE_DATA + AX_CW_COMPLIANCE_MARGIN + CWCMargin + AX_CCW_COMPLIANCE_MARGIN + CCWCMargin)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_CM_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_CW_COMPLIANCE_MARGIN);
  sendData(CWCMargin);
  sendData(AX_CCW_COMPLIANCE_MARGIN);
  sendData(CCWCMargin);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error());
}

int DynamixelClass::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope)
{
  Checksum = (~(ID + AX_CS_LENGTH + AX_WRITE_DATA + AX_CW_COMPLIANCE_SLOPE + CWCSlope + AX_CCW_COMPLIANCE_SLOPE + CCWCSlope)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_CS_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_CW_COMPLIANCE_SLOPE);
  sendData(CWCSlope);
  sendData(AX_CCW_COMPLIANCE_SLOPE);
  sendData(CCWCSlope);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error());
}

int DynamixelClass::setPunch(unsigned char ID, int Punch)
{
  char Punch_H, Punch_L;
  Punch_H = Punch >> 8; // 16 bits - 2 x 8 bits variables
  Punch_L = Punch;
  Checksum = (~(ID + AX_PUNCH_LENGTH + AX_WRITE_DATA + AX_PUNCH_L + Punch_L + Punch_H)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_PUNCH_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_PUNCH_L);
  sendData(Punch_L);
  sendData(Punch_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::moving(unsigned char ID)
{
  Checksum = (~(ID + AX_MOVING_LENGTH + AX_READ_DATA + AX_MOVING + AX_BYTE_READ)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_MOVING_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_MOVING);
  sendData(AX_BYTE_READ);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  Moving_Byte = -1;
  Time_Counter = 0;
  while ((availableData() < 6) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }

  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
        return (Error_Byte * (-1));
      Moving_Byte = readData(); // Temperature
    }
  }
  return (Moving_Byte); // Returns the read temperature
}

int DynamixelClass::lockRegister(unsigned char ID)
{
  Checksum = (~(ID + AX_LR_LENGTH + AX_WRITE_DATA + AX_LOCK + LOCK)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(ID);
  sendData(AX_LR_LENGTH);
  sendData(AX_WRITE_DATA);
  sendData(AX_LOCK);
  sendData(LOCK);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelClass::RWStatus(unsigned char ID)
{
  Checksum = (~(ID + AX_RWS_LENGTH + AX_READ_DATA + AX_REGISTERED_INSTRUCTION + AX_BYTE_READ)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_RWS_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_REGISTERED_INSTRUCTION);
  sendData(AX_BYTE_READ);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  RWS_Byte = -1;
  Time_Counter = 0;
  while ((availableData() < 6) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }

  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
        return (Error_Byte * (-1));
      RWS_Byte = readData(); // Temperature
    }
  }
  return (RWS_Byte); // Returns the read temperature
}

int DynamixelClass::readSpeed(unsigned char ID)
{
  Checksum = (~(ID + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_BYTE_READ_POS)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_POS_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_PRESENT_SPEED_L);
  sendData(AX_BYTE_READ_POS);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  Speed_Long_Byte = -1;
  Time_Counter = 0;
  while ((availableData() < 7) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }

  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
        return (Error_Byte * (-1));

      Speed_Low_Byte = readData(); // Position Bytes
      Speed_High_Byte = readData();
      Speed_Long_Byte = Speed_High_Byte << 8;
      Speed_Long_Byte = Speed_Long_Byte + Speed_Low_Byte;
    }
  }
  return (Speed_Long_Byte); // Returns the read position
}

int DynamixelClass::readLoad(unsigned char ID)
{
  Checksum = (~(ID + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_BYTE_READ_POS)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START);
  sendData(AX_START);
  sendData(ID);
  sendData(AX_POS_LENGTH);
  sendData(AX_READ_DATA);
  sendData(AX_PRESENT_LOAD_L);
  sendData(AX_BYTE_READ_POS);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  Load_Long_Byte = -1;
  Time_Counter = 0;
  while ((availableData() < 7) & (Time_Counter < TIME_OUT))
  {
    Time_Counter++;
    delayus(1);
  }

  while (availableData() > 0)
  {
    Incoming_Byte = readData();
    if ((Incoming_Byte == 255) & (peekData() == 255))
    {
      readData();                         // Start Bytes
      readData();                         // Ax-12 ID
      readData();                         // Length
      if ((Error_Byte = readData()) != 0) // Error
        return (Error_Byte * (-1));

      Load_Low_Byte = readData(); // Position Bytes
      Load_High_Byte = readData();
      Load_Long_Byte = Load_High_Byte << 8;
      Load_Long_Byte = Load_Long_Byte + Load_Low_Byte;
    }
  }
  return (Load_Long_Byte); // Returns the read position
}

int DynamixelClass::synWritePos(unsigned char ID1, int Position1, unsigned char ID2, int Position2)
{
  char Position1_H = Position1 >> 8; // 16 bits - 2 x 8 bits variables
  char Position1_L = Position1;
  char Position2_H = Position2 >> 8; // 16 bits - 2 x 8 bits variables
  char Position2_L = Position2;
  Checksum = (~(BROADCAST_ID + 10 + AX_SYNC_WRITE + AX_GOAL_POSITION_L + 2 + ID1 + Position1_L + Position1_H + ID2 + Position2_L + Position2_H)) & 0xFF;

  switchCom(Direction_Pin, Tx_MODE);
  sendData(AX_START); // Send Instructions over Serial
  sendData(AX_START);
  sendData(BROADCAST_ID);

  sendData(10); //Length = (L+1) X N + 4   (L: Data Length per RX-64, N: the number of RX-64s)
  sendData(AX_SYNC_WRITE);
  sendData(AX_GOAL_POSITION_L);
  sendData(2); // number of "sync parameters" for each motor
  sendData(ID1);
  sendData(Position1_L); //"sync parameter"
  sendData(Position1_H); //"sync parameter"
  sendData(ID2);
  sendData(Position2_L);
  sendData(Position2_H);
  sendData(Checksum);
  axflush();
  delayus(TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  // return (read_error());                 // No Return  with synWrite
}

// Public Methods //////////////////////////////////////////////////////////////
//Protocol 2.0

DynamixelXClass::DynamixelXClass(Stream &serialx)
    : serialx_(serialx) {}

void DynamixelXClass::SetDirPin(unsigned char directionPin)
{
  Direction_Pin = directionPin;
  setDPin(Direction_Pin, OUTPUT);
}

void DynamixelXClass::begin(long baud, unsigned char directionPin)
{
  Direction_Pin = directionPin;
  setDPin(Direction_Pin, OUTPUT);
  //beginComXm(baud);
}

struct ReturnPacket2 DynamixelXClass::ping(unsigned char ID)
{
  ptr_packetsize0 = &packetsize0;
  packetsize0.id = ID;
  packetsize0.instruction = XM_PING;
  packetsize0.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize0.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize0, 8); //init crc_accum | packet | 5+Packet Lenght()
  packetsize0.crc_L = (CRC & 0x00FF);
  packetsize0.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize0, 10); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  //Reading of packets returned
  ptr_returnpacket2 = &returnpacket2;

  while (availableDataXm())
  {
    Incoming_Byte = readDataXm(); //FF
    if ((Incoming_Byte == 255) & (peekDataXm() == 255))
    {
      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FD
      readDataXm();              // reserved
      readDataXm();              // ID
      readDataXm();              // Length L
      readDataXm();              // Length H
      readDataXm();              // Instruction
      Error_Byte = readDataXm(); // Error
      if (Error_Byte != 0)       // Error
          //return (Error_Byte*(-1));
        Model_Low_Byte = readDataXm(); // Position Bytes Low
      Model_High_Byte = readDataXm();  // Position Bytes High
      Model_Long_Byte = Model_High_Byte << 8;
      returnpacket2.value1 = Model_Long_Byte + Model_Low_Byte;
      returnpacket2.value2 = readDataXm();
      readDataXm(); //CRC
      readDataXm(); //CRC

      //returnpacket2.value1 = Position_Long_Byte;

      //return (*(uint32_t*)&returnpacket2);     // Returns the read position
    }
  }
}

int DynamixelXClass::move(unsigned char ID, int Position)
{
  ptr_packetsize6 = &packetsize6;
  //TxPacket[16]={XM_START_FF,XM_START_FF,XM_START_FD,XM_0,ID,XM_GOAL_LENGTH,XM_0,XM_WRITE,XM_GOAL_POSITION,XM_0,Position_L,Position_H,XM_0,XM_0,crc_L,crc_H};
  packetsize6.id = ID;
  packetsize6.instruction = XM_WRITE;
  packetsize6.param1 = XM_GOAL_POSITION;
  packetsize6.param3 = (Position & 0x00FF);
  packetsize6.param4 = (Position >> 8) & 0x00FF;
  packetsize6.param5 = XM_0; //value
  packetsize6.param6 = XM_0; //value
  packetsize6.crc_L = 0x00;  //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize6.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize6, 14); //(init crc_accum | packet | 5+Packet Lenght())
  packetsize6.crc_L = (CRC & 0x00FF);                                 //MAJ CRC avec valeur calculée
  packetsize6.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize6, 16); //Size of the packet
  xmflush();                                 //wait the end of the transmission
  delayus(XM_TX_DELAY_TIME);                 //wait before enable the response of the dynamixel
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelXClass::setBD(unsigned char ID, int baud)
{
  ptr_packetsize3 = &packetsize3;
  // TxPacket[13]={XM_START_FF,XM_START_FF,XM_START_FD,XM_0,ID,XM_BD_LENGTH,XM_0,XM_WRITE,XM_BAUD_RATE,XM_0,baud,XM_0,XM_0};
  packetsize3.id = ID;
  packetsize3.instruction = XM_WRITE;
  packetsize3.param1 = XM_BAUD_RATE;
  packetsize3.param3 = baud;
  packetsize3.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize3.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize3, 11); //init crc_accum | packet | 5+Packet Lenght()
  packetsize3.crc_L = (CRC & 0x00FF);
  packetsize3.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize3, 13); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelXClass::setTorque(unsigned char ID, bool torque)
{
  ptr_packetsize3 = &packetsize3;
  //TxPacket[13]={XM_START_FF,XM_START_FF,XM_START_FD,XM_0,ID,XM_TORQUE_LENGTH,XM_0,XM_WRITE,XM_TORQUE_ENABLE ,XM_0,torque,XM_0,XM_0};
  packetsize3.id = ID;
  packetsize3.instruction = XM_WRITE;
  packetsize3.param1 = XM_TORQUE_ENABLE;
  packetsize3.param3 = torque;
  packetsize3.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize3.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize3, 11); //init crc_accum | packet | 5+Packet Lenght()
  packetsize3.crc_L = (CRC & 0x00FF);
  packetsize3.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize3, 13); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelXClass::setRDT(unsigned char ID, unsigned char RDT) // SET return delay time
{
  ptr_packetsize3 = &packetsize3;
  //TxPacket[13]={XM_START_FF,XM_START_FF,XM_START_FD,XM_0,ID,XM_RDT_LENGTH,XM_0,XM_WRITE,XM_RETURN_DELAY_TIME,XM_0,RDT,XM_0,XM_0};
  packetsize3.id = ID;
  packetsize3.instruction = XM_WRITE;
  packetsize3.param1 = XM_RETURN_DELAY_TIME;
  packetsize3.param3 = RDT;
  packetsize3.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize3.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize3, 11); //init crc_accum | packet | 5+Packet Lenght()
  packetsize3.crc_L = (CRC & 0x00FF);
  packetsize3.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize3, 13); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelXClass::ledStatus(unsigned char ID, bool Status)
{
  ptr_packetsize3 = &packetsize3;
  packetsize3.id = ID;
  packetsize3.instruction = XM_WRITE;
  packetsize3.param1 = XM_LED;
  packetsize3.param3 = Status;
  packetsize3.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize3.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize3, 11); //init crc_accum | packet | 5+Packet Lenght()
  packetsize3.crc_L = (CRC & 0x00FF);
  packetsize3.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize3, 13); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  return (read_error()); // Return the read error
}

int DynamixelXClass::synWritePos(unsigned char ID1, int Position1, unsigned char ID2, int Position2)
{
  ptr_packetsync14 = &packetsync14;
  packetsync14.param1 = XM_GOAL_POSITION;
  packetsync14.param3 = XM_SYNC_WRITE_LENGTH;
  packetsync14.param4 = XM_0;
  packetsync14.param5 = ID1;                       //Parameter 5
  packetsync14.param6 = (Position1 & 0x00FF);      //Parameter 6
  packetsync14.param7 = (Position1 >> 8) & 0x00FF; //Parameter 7
  packetsync14.param8 = XM_0;                      //Parameter 8
  packetsync14.param9 = XM_0;                      //Parameter 9
  packetsync14.param10 = ID2;                      //Parameter 10
  packetsync14.param11 = (Position2 & 0x00FF);     //Parameter 11
  packetsync14.param12 = (Position2 >> 8) & 0x00FF;
  ;                            //Parameter 12
  packetsync14.param13 = XM_0; //Parameter 13
  packetsync14.param14 = XM_0; //Parameter 14
  packetsync14.crc_L = 0x00;   //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsync14.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsync14, 22); //init crc_accum | packet | 5+Packet Lenght()
  packetsync14.crc_L = (CRC & 0x00FF);
  packetsync14.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsync14, 24); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  // return (read_error());                 // No Return  with synWrite
}

int DynamixelXClass::readPosition(unsigned char ID)
{
  ptr_packetsize4 = &packetsize4;
  packetsize4.id = ID;
  packetsize4.instruction = XM_READ;
  packetsize4.param1 = XM_PRESENT_POSITION;
  packetsize4.param3 = LENGTH_4;
  packetsize4.param4 = XM_0;
  packetsize4.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize4.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize4, 12); //init crc_accum | packet | 5+Packet Lenght()
  packetsize4.crc_L = (CRC & 0x00FF);
  packetsize4.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize4, 14); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  Load_Long_Byte = -1;
  Time_Counter = 0;
  while ((availableDataXm() < 11) & (Time_Counter < TIME_OUT))
  { // Wait the end of the return packet
    Time_Counter++;
    delayus(1);
  }

  while (availableDataXm())
  {
    Incoming_Byte = readDataXm(); //FF
    if ((Incoming_Byte == 255) & (peekDataXm() == 255))
    {
      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FD
      readDataXm();              // reserved
      readDataXm();              // ID
      readDataXm();              // Length L
      readDataXm();              // Length H
      readDataXm();              // Instruction
      Error_Byte = readDataXm(); // Error
      if (Error_Byte != 0)       // Error
        return (Error_Byte * (-1));
      Load_Low_Byte = readDataXm(); // Position Bytes
      Load_High_Byte = readDataXm();
      Load_Long_Byte = Load_High_Byte << 8;
      Load_Long_Byte = Load_Long_Byte + Load_Low_Byte;

      return (Load_Long_Byte); // Returns the read position
    }
  }
}

int DynamixelXClass::readPosition(unsigned char ID, int *Pos_Long_Byte)
{
  ptr_packetsize4 = &packetsize4;
  packetsize4.id = ID;
  packetsize4.instruction = XM_READ;
  packetsize4.param1 = XM_PRESENT_POSITION;
  packetsize4.param3 = LENGTH_4;
  packetsize4.param4 = XM_0;
  packetsize4.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize4.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize4, 12); //init crc_accum | packet | 5+Packet Lenght()
  packetsize4.crc_L = (CRC & 0x00FF);
  packetsize4.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize4, 14); //Size of the packet
  xmflush();
  delayus(XM_TX_DELAY_TIME);
  switchCom(Direction_Pin, Rx_MODE);

  *Pos_Long_Byte = -1;
  Time_Counter = 0;

  while ((availableDataXm() < 11) & (Time_Counter < TIME_OUT))
  { // Wait the end of the return packet
    Time_Counter++;
    delayus(1);
  }

  while (availableDataXm())
  {
    Incoming_Byte = readDataXm(); //FF
    if ((Incoming_Byte == 255) & (peekDataXm() == 255))
    {
      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FD
      readDataXm();              // reserved
      readDataXm();              // ID
      readDataXm();              // Length L
      readDataXm();              // Length H
      readDataXm();              // Instruction
      Error_Byte = readDataXm(); // Error
      if (Error_Byte != 0)
      {
        return (Error_Byte);
      }
      Position_Low_Byte = readDataXm(); // Position Bytes
      Position_High_Byte = readDataXm();
      *Pos_Long_Byte = Position_High_Byte << 8 | Position_Low_Byte;
    }
  }
}

int DynamixelXClass::syncReadPos(unsigned char ID1, int *Pos_Long_Byte1, unsigned char ID2, int *Pos_Long_Byte2)
{
  ptr_packetsize6 = &packetsize6;
  packetsize6.id = BROADCAST_ID;
  packetsize6.instruction = XM_SYNC_READ;
  packetsize6.param1 = XM_PRESENT_POSITION;
  packetsize6.param3 = LENGTH_4;
  packetsize6.param4 = XM_0;
  packetsize6.param5 = ID1;
  packetsize6.param6 = ID2;
  packetsize6.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize6.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize6, 14); //(init crc_accum | packet | 5+Packet Lenght())
  packetsize6.crc_L = (CRC & 0x00FF);                                 //MAJ CRC avec valeur calculée
  packetsize6.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize6, 16); //Size of the packet
  xmflush();                                 //wait the end of the transmission
  delayus(XM_TX_DELAY_TIME);                 //wait before enable the response of the dynamixel
  switchCom(Direction_Pin, Rx_MODE);

  *Pos_Long_Byte1 = -1;
  *Pos_Long_Byte2 = -1;
  detectID = 0;
  Time_Counter = 0;
  while ((availableDataXm() < 28) & (Time_Counter < 2 * TIME_OUT))
  { // Wait the end of the return of 2 packets
    Time_Counter++;
    delayus(1);
  }

  while (availableDataXm())
  {
    Incoming_Byte = readDataXm(); //FF
    if ((Incoming_Byte == 255) & (peekDataXm() == 255))
    {
      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FD
      readDataXm();              // reserved
      detectID = readDataXm();   // ID1
      readDataXm();              // Length L
      readDataXm();              // Length H
      readDataXm();              // Instruction
      Error_Byte = readDataXm(); // Error
      if (Error_Byte != 0)
      {
        return (Error_Byte);
      }
      Position_Low_Byte = readDataXm();  // Position Bytes Low
      Position_High_Byte = readDataXm(); // Position Bytes High
      //*Pos_Long_Byte1 = Position_High_Byte << 8 | Position_Low_Byte;
      readDataXm();
      readDataXm();
      readDataXm(); //CRC
      readDataXm(); //CRC
      if (detectID == ID1)
      {
        *Pos_Long_Byte1 = Position_High_Byte << 8 | Position_Low_Byte;
      }

      if (detectID == ID2)
      {
        *Pos_Long_Byte2 = Position_High_Byte << 8 | Position_Low_Byte;
      }

      else
        return (Error_Byte * (-1));
    }
  }
}

int DynamixelXClass::syncReadCur(unsigned char ID1, unsigned char ID2)
{
  ptr_packetsize6 = &packetsize6;
  packetsize6.id = BROADCAST_ID;
  packetsize6.instruction = XM_SYNC_READ;
  packetsize6.param1 = XM_PRESENT_CURRENT;
  packetsize6.param3 = LENGTH_2;
  packetsize6.param4 = XM_0;
  packetsize6.param5 = ID1;
  packetsize6.param6 = ID2;
  packetsize6.crc_L = 0x00; //(pour son calcul les valeurs du CRC doivent être misent à 0 dans le packet)
  packetsize6.crc_H = 0x00;
  //calcul CRC
  unsigned short CRC = update_crc(0, (uint8_t *)ptr_packetsize6, 14); //(init crc_accum | packet | 5+Packet Lenght())
  packetsize6.crc_L = (CRC & 0x00FF);                                 //MAJ CRC avec valeur calculée
  packetsize6.crc_H = (CRC >> 8) & 0x00FF;
  //Send Data
  switchCom(Direction_Pin, Tx_MODE);
  sendDataX((uint8_t *)ptr_packetsize6, 16); //Size of the packet
  xmflush();                                 //wait the end of the transmission
  delayus(XM_TX_DELAY_TIME);                 //wait before enable the response of the dynamixel
  switchCom(Direction_Pin, Rx_MODE);

  Load_Long_Byte = -1;
  Time_Counter = 0;
  while ((availableDataXm() < 28) & (Time_Counter < 2 * TIME_OUT))
  { // Wait the end of the return of 2 packets
    Time_Counter++;
    delayus(1);
  }
  //Reading of packets returned
  ptr_returnpacket2 = &returnpacket2;

  while (availableDataXm())
  {
    Incoming_Byte = readDataXm(); //FF
    if ((Incoming_Byte == 255) & (peekDataXm() == 255))
    {
      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FD
      readDataXm();              // reserved
      detectID = readDataXm();   // ID
      readDataXm();              // Length L
      readDataXm();              // Length H
      readDataXm();              // Instruction
      Error_Byte = readDataXm(); // Error
      if (Error_Byte != 0)       // Error
        return (Error_Byte * (-1));
      Position_Low_Byte = readDataXm();  // Position Bytes Low
      Position_High_Byte = readDataXm(); // Position Bytes High
      Position_Long_Byte = Position_High_Byte << 8;
      Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
      readDataXm(); //CRC
      readDataXm(); //CRC

      returnpacket2.value1 = Position_Long_Byte;

      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FF
      readDataXm();              // Start Bytes FD
      readDataXm();              // reserved
      readDataXm();              // ID
      readDataXm();              // Length L
      readDataXm();              // Length H
      readDataXm();              // Instruction
      Error_Byte = readDataXm(); // Error
      if (Error_Byte != 0)       // Error
        return (Error_Byte * (-1));
      Position_Low_Byte = readDataXm();  // Position Bytes Low
      Position_High_Byte = readDataXm(); // Position Bytes High
      Position_Long_Byte = Position_High_Byte << 8;
      Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
      readDataXm(); //CRC
      readDataXm(); //CRC

      returnpacket2.value2 = Position_Long_Byte;

      return (*(uint32_t *)&returnpacket2); // Returns the read position
    }
  }
}

//DynamixelClass Dynamixel;
//DynamixelXClass DynamixelX;
