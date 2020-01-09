/*
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

 */

#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "RX24F.h"

// Macro for the selection of the Serial Port

#define sendData(args)  (Serial.write(args))    // Write Over Serial
#define availableData() (Serial.available())    // Check Serial Data Available
#define readData()      (Serial.read())         // Read Serial Data
#define peekData()      (Serial.peek())         // Peek Serial Data
#define beginCom(args)  (Serial.begin(args))    // Begin Serial Comunication
#define endCom()        (Serial.end())          // End Serial Comunication
#define flush()         (Serial.flush())        // Wait until write has finished

// Macro for Timing

#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds

// Macro for Comunication Flow Control

#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode))       // Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode


// Private Methods //////////////////////////////////////////////////////////////

int DynamixelClass::read_error(void)
{
	Time_Counter = 0;
	while((availableData() < 5) & (Time_Counter < TIME_OUT)){  // Wait for Data
		Time_Counter++;
		delayus(1000);
	}

	while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                                    // Start Bytes
			readData();                                    // Ax-12 ID
			readData();                                    // Length
			Error_Byte = readData();                       // Error
				return (Error_Byte);
		}
	}
	return (-1);											 // No RX24F Response
}

// Public Methods //////////////////////////////////////////////////////////////

void DynamixelClass::begin(long baud, unsigned char directionPin)
{
	Direction_Pin = directionPin;
	setDPin(Direction_Pin,OUTPUT);
	beginCom(baud);
}

void DynamixelClass::begin(long baud)
{
	beginCom(baud);
}

void DynamixelClass::end()
{
	endCom();
}

int DynamixelClass::reset(unsigned char ID)
{
	Checksum = (~(ID + RX_RESET_LENGTH + RX_INSTR_RESET))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(RX_START);
	sendData(RX_START);
	sendData(ID);
	sendData(RX_RESET_LENGTH);
	sendData(RX_INSTR_RESET);
	sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);


    return (read_error());
}

int DynamixelClass::ping(unsigned char ID)
{
	Checksum = (~(ID + RX_INSTR_READ_DATA + RX_INSTR_PING))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(RX_START);
	sendData(RX_START);
	sendData(ID);
	sendData(RX_INSTR_READ_DATA);
	sendData(RX_INSTR_PING);
	sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());
}

int DynamixelClass::setID(unsigned char ID, unsigned char newID)
{
	Checksum = (~(ID + RX_ID_LENGTH + RX_INSTR_WRITE_DATA + RX_ID + newID))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
	sendData(RX_ID_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_ID);
    sendData(newID);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);\

    return (read_error());                // Return the read error
}

int DynamixelClass::setBD(unsigned char ID, long baud)
{
	unsigned char Baud_Rate = (2000000/baud) - 1;
    Checksum = (~(ID + RX_BD_LENGTH + RX_INSTR_WRITE_DATA + RX_BAUD_RATE + Baud_Rate))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                 // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
	sendData(RX_BD_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_BAUD_RATE);
    sendData(Baud_Rate);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    return (read_error());                // Return the read error
}

int DynamixelClass::move(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = (Position >> 8) & 0xFF;           // 16 bits - 2 x 8 bits variables
    Position_L = Position & 0xFF;
	Checksum = (~(ID + RX_GOAL_LENGTH + RX_INSTR_WRITE_DATA + RX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                 // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_GOAL_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                 // Return the read error
}

int DynamixelClass::moveSpeed(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
	Checksum = (~(ID + RX_GOAL_SP_LENGTH + RX_INSTR_WRITE_DATA + RX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_GOAL_SP_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    return (read_error());               // Return the read error
}

int DynamixelClass::setEndless(unsigned char ID, bool Status)
{
 if ( Status ) {
	  char RX_CCW_AL_LT = 0;     // Changing the CCW Angle Limits for Full Rotation.
	  Checksum = (~(ID + RX_GOAL_LENGTH + RX_INSTR_WRITE_DATA + RX_CCW_ANGLE_LIMIT_L))&0xFF;

	  switchCom(Direction_Pin,Tx_MODE);
      sendData(RX_START);                // Send Instructions over Serial
      sendData(RX_START);
      sendData(ID);
      sendData(RX_GOAL_LENGTH);
      sendData(RX_INSTR_WRITE_DATA);
      sendData(RX_CCW_ANGLE_LIMIT_L );
      sendData(RX_CCW_AL_LT);
      sendData(RX_CCW_AL_LT);
      sendData(Checksum);
   flush();
   switchCom(Direction_Pin,Rx_MODE);
      delayus(TX_DELAY_TIME);

	  return(read_error());
 }
 else
 {
	 turn(ID,0,0);
	 Checksum = (~(ID + RX_GOAL_LENGTH + RX_INSTR_WRITE_DATA + RX_CCW_ANGLE_LIMIT_L + RX_CCW_AL_L + RX_CCW_AL_H))&0xFF;

	 switchCom(Direction_Pin,Tx_MODE);
	 sendData(RX_START);                 // Send Instructions over Serial
	 sendData(RX_START);
	 sendData(ID);
	 sendData(RX_GOAL_LENGTH);
	 sendData(RX_INSTR_WRITE_DATA);
	 sendData(RX_CCW_ANGLE_LIMIT_L);
	 sendData(RX_CCW_AL_L);
	 sendData(RX_CCW_AL_H);
	 sendData(Checksum);
   flush();
   switchCom(Direction_Pin,Rx_MODE);
	 delayus(TX_DELAY_TIME);

	 return (read_error());                 // Return the read error
  }
 }

int DynamixelClass::turn(unsigned char ID, bool SIDE, int Speed)
{
		if (SIDE == LEFT){                          // Move Left///////////////////////////

			char Speed_H,Speed_L;
			Speed_H = Speed >> 8;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			Checksum = (~(ID + RX_SPEED_LENGTH + RX_INSTR_WRITE_DATA + RX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;

			switchCom(Direction_Pin,Tx_MODE);
			sendData(RX_START);                // Send Instructions over Serial
			sendData(RX_START);
			sendData(ID);
			sendData(RX_SPEED_LENGTH);
			sendData(RX_INSTR_WRITE_DATA);
			sendData(RX_GOAL_SPEED_L);
			sendData(Speed_L);
			sendData(Speed_H);
			sendData(Checksum);
      flush();
      switchCom(Direction_Pin,Rx_MODE);
			delayus(TX_DELAY_TIME);

			return(read_error());               // Return the read error
		}
		else
		{                                            // Move Rigth////////////////////
			char Speed_H,Speed_L;
			Speed_H = (Speed >> 8) + 4;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			Checksum = (~(ID + RX_SPEED_LENGTH + RX_INSTR_WRITE_DATA + RX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;

			switchCom(Direction_Pin,Tx_MODE);
			sendData(RX_START);                // Send Instructions over Serial
			sendData(RX_START);
			sendData(ID);
			sendData(RX_SPEED_LENGTH);
			sendData(RX_INSTR_WRITE_DATA);
			sendData(RX_GOAL_SPEED_L);
			sendData(Speed_L);
			sendData(Speed_H);
			sendData(Checksum);
      flush();
      switchCom(Direction_Pin,Rx_MODE);
			delayus(TX_DELAY_TIME);

			return(read_error());                // Return the read error
		}
}

int DynamixelClass::moveRW(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;
    Checksum = (~(ID + RX_GOAL_LENGTH + RX_INSTR_REG_WRITE + RX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                 // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_GOAL_LENGTH);
    sendData(RX_INSTR_REG_WRITE);
    sendData(RX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                 // Return the read error
}

int DynamixelClass::moveSpeedRW(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
    Checksum = (~(ID + RX_GOAL_SP_LENGTH + RX_INSTR_REG_WRITE + RX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_GOAL_SP_LENGTH);
    sendData(RX_INSTR_REG_WRITE);
    sendData(RX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    return (read_error());               // Return the read error
}

void DynamixelClass::action()
{
	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(BROADCAST_ID);
    sendData(RX_ACTION_LENGTH);
    sendData(RX_INSTR_ACTION);
    sendData(RX_ACTION_CHECKSUM);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);
}

int DynamixelClass::torqueStatus( unsigned char ID, bool Status)
{
    Checksum = (~(ID + RX_TORQUE_LENGTH + RX_INSTR_WRITE_DATA + RX_TORQUE_ENABLE + Status))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);              // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_TORQUE_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_TORQUE_ENABLE);
    sendData(Status);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    return (read_error());              // Return the read error
}

int DynamixelClass::ledStatus(unsigned char ID, bool Status)
{
    Checksum = (~(ID + RX_LED_LENGTH + RX_INSTR_WRITE_DATA + RX_LED + Status))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);              // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_LED_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_LED);
    sendData(Status);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    return (read_error());              // Return the read error
}

int DynamixelClass::readTemperature(unsigned char ID)
{
    Checksum = (~(ID + RX_TEM_LENGTH  + RX_INSTR_READ_DATA + RX_PRESENT_TEMPERATURE + RX_BYTE_READ))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);
    sendData(RX_START);
    sendData(ID);
    sendData(RX_TEM_LENGTH);
    sendData(RX_INSTR_READ_DATA);
    sendData(RX_PRESENT_TEMPERATURE);
    sendData(RX_BYTE_READ);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    Temperature_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }

    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Temperature_Byte = readData();         // Temperature
		}
    }
	return (Temperature_Byte);               // Returns the read temperature
}

int DynamixelClass::readPosition(unsigned char ID)
{
    Checksum = (~(ID + RX_POS_LENGTH  + RX_INSTR_READ_DATA + RX_PRESENT_POSITION_L + RX_BYTE_READ_POS))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);
    sendData(RX_START);
    sendData(ID);
    sendData(RX_POS_LENGTH);
    sendData(RX_INSTR_READ_DATA);
    sendData(RX_PRESENT_POSITION_L);
    sendData(RX_BYTE_READ_POS);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    Position_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }

    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));

			Position_Low_Byte = readData();            // Position Bytes
			Position_High_Byte = readData();
			Position_Long_Byte = Position_High_Byte << 8;
			Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
		}
    }
	return (Position_Long_Byte);     // Returns the read position
}

int DynamixelClass::readVoltage(unsigned char ID)
{
    Checksum = (~(ID + RX_VOLT_LENGTH  + RX_INSTR_READ_DATA + RX_PRESENT_VOLTAGE + RX_BYTE_READ))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);
    sendData(RX_START);
    sendData(ID);
    sendData(RX_VOLT_LENGTH);
    sendData(RX_INSTR_READ_DATA);
    sendData(RX_PRESENT_VOLTAGE);
    sendData(RX_BYTE_READ);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    Voltage_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }

    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Voltage_Byte = readData();             // Voltage
		}
    }
	return (Voltage_Byte);               // Returns the read Voltage
}

int DynamixelClass::setTempLimit(unsigned char ID, unsigned char Temperature)
{
	Checksum = (~(ID + RX_TL_LENGTH + RX_INSTR_WRITE_DATA + RX_LIMIT_TEMPERATURE + Temperature))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(RX_START);
	sendData(RX_START);
	sendData(ID);
	sendData(RX_TL_LENGTH);
	sendData(RX_INSTR_WRITE_DATA);
	sendData(RX_LIMIT_TEMPERATURE);
    sendData(Temperature);
	sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());
}

int DynamixelClass::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage)
{
	Checksum = (~(ID + RX_VL_LENGTH +RX_INSTR_WRITE_DATA+ RX_LOW_LIMIT_VOLTAGE + DVoltage + UVoltage))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(RX_START);
	sendData(RX_START);
	sendData(ID);
	sendData(RX_VL_LENGTH);
	sendData(RX_INSTR_WRITE_DATA);
	sendData(RX_LOW_LIMIT_VOLTAGE);
    sendData(DVoltage);
    sendData(UVoltage);
	sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());
}

int DynamixelClass::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit)
{
	char CW_H,CW_L,CCW_H,CCW_L;
    CW_H = CWLimit >> 8;
    CW_L = CWLimit;                // 16 bits - 2 x 8 bits variables
    CCW_H = CCWLimit >> 8;
    CCW_L = CCWLimit;
	Checksum = (~(ID + RX_VL_LENGTH +RX_INSTR_WRITE_DATA+ RX_CW_ANGLE_LIMIT_L + CW_H + CW_L + RX_CCW_ANGLE_LIMIT_L + CCW_H + CCW_L))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(RX_START);
	sendData(RX_START);
	sendData(ID);
	sendData(RX_CCW_CW_LENGTH);
	sendData(RX_INSTR_WRITE_DATA);
	sendData(RX_CW_ANGLE_LIMIT_L);
    sendData(CW_L);
	sendData(CW_H);
	sendData(RX_CCW_ANGLE_LIMIT_L);
    sendData(CCW_L);
	sendData(CCW_H);
	sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());
}

int DynamixelClass::setMaxTorque(unsigned char ID, int MaxTorque)
{
    char MaxTorque_H,MaxTorque_L;
    MaxTorque_H = MaxTorque >> 8;           // 16 bits - 2 x 8 bits variables
    MaxTorque_L = MaxTorque;
	Checksum = (~(ID + RX_MT_LENGTH + RX_INSTR_WRITE_DATA + RX_MAX_TORQUE_L + MaxTorque_L + MaxTorque_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                 // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_MT_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_MAX_TORQUE_L);
    sendData(MaxTorque_L);
    sendData(MaxTorque_H);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                 // Return the read error
}

int DynamixelClass::setSRL(unsigned char ID, unsigned char SRL)
{
	Checksum = (~(ID + RX_SRL_LENGTH + RX_INSTR_WRITE_DATA + RX_STATUS_RETURN_LEVEL + SRL))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
	sendData(RX_SRL_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_STATUS_RETURN_LEVEL);
    sendData(SRL);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                // Return the read error
}

int DynamixelClass::setRDT(unsigned char ID, unsigned char RDT)
{
	Checksum = (~(ID + RX_RDT_LENGTH + RX_INSTR_WRITE_DATA + RX_RETURN_DELAY_TIME + (RDT/2)))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
	sendData(RX_RDT_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_RETURN_DELAY_TIME);
    sendData((RDT/2));
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                // Return the read error
}

int DynamixelClass::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm)
{
	Checksum = (~(ID + RX_LEDALARM_LENGTH + RX_INSTR_WRITE_DATA + RX_ALARM_LED + LEDAlarm))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
	sendData(RX_LEDALARM_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_ALARM_LED);
    sendData(LEDAlarm);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                // Return the read error
}

int DynamixelClass::setShutdownAlarm(unsigned char ID, unsigned char SALARM)
{
	Checksum = (~(ID + RX_SALARM_LENGTH + RX_ALARM_SHUTDOWN + RX_ALARM_LED + SALARM))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
	sendData(RX_SALARM_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_ALARM_SHUTDOWN);
    sendData(SALARM);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                // Return the read error
}

int DynamixelClass::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin)
{
	Checksum = (~(ID + RX_CM_LENGTH +RX_INSTR_WRITE_DATA+ RX_CW_COMPLIANCE_MARGIN + CWCMargin + RX_CCW_COMPLIANCE_MARGIN + CCWCMargin))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(RX_START);
	sendData(RX_START);
	sendData(ID);
	sendData(RX_CM_LENGTH);
	sendData(RX_INSTR_WRITE_DATA);
	sendData(RX_CW_COMPLIANCE_MARGIN);
    sendData(CWCMargin);
	sendData(RX_CCW_COMPLIANCE_MARGIN);
    sendData(CCWCMargin);
	sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());
}

int DynamixelClass::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope)
{
	Checksum = (~(ID + RX_CS_LENGTH +RX_INSTR_WRITE_DATA+ RX_CW_COMPLIANCE_SLOPE + CWCSlope + RX_CCW_COMPLIANCE_SLOPE + CCWCSlope))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(RX_START);
	sendData(RX_START);
	sendData(ID);
	sendData(RX_CS_LENGTH);
	sendData(RX_INSTR_WRITE_DATA);
	sendData(RX_CW_COMPLIANCE_SLOPE);
    sendData(CWCSlope);
	sendData(RX_CCW_COMPLIANCE_SLOPE);
    sendData(CCWCSlope);
	sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());
}

int DynamixelClass::setPunch(unsigned char ID, int Punch)
{
    char Punch_H,Punch_L;
    Punch_H = Punch >> 8;           // 16 bits - 2 x 8 bits variables
    Punch_L = Punch;
	Checksum = (~(ID + RX_PUNCH_LENGTH + RX_INSTR_WRITE_DATA + RX_PUNCH_L + Punch_L + Punch_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                 // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
    sendData(RX_PUNCH_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_PUNCH_L);
    sendData(Punch_L);
    sendData(Punch_H);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                 // Return the read error
}

int DynamixelClass::moving(unsigned char ID)
{
    Checksum = (~(ID + RX_MOVING_LENGTH  + RX_INSTR_READ_DATA + RX_MOVING + RX_BYTE_READ))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);
    sendData(RX_START);
    sendData(ID);
    sendData(RX_MOVING_LENGTH);
    sendData(RX_INSTR_READ_DATA);
    sendData(RX_MOVING);
    sendData(RX_BYTE_READ);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    Moving_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }

    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Moving_Byte = readData();         // Temperature
		}
    }
	return (Moving_Byte);               // Returns the read temperature
}

int DynamixelClass::lockRegister(unsigned char ID)
{
	Checksum = (~(ID + RX_LR_LENGTH + RX_INSTR_WRITE_DATA + RX_LOCK + LOCK))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);                // Send Instructions over Serial
    sendData(RX_START);
    sendData(ID);
	sendData(RX_LR_LENGTH);
    sendData(RX_INSTR_WRITE_DATA);
    sendData(RX_LOCK);
    sendData(LOCK);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
	delayus(TX_DELAY_TIME);

    return (read_error());                // Return the read error
}

int DynamixelClass::RWStatus(unsigned char ID)
{
    Checksum = (~(ID + RX_RWS_LENGTH  + RX_INSTR_READ_DATA + RX_INSTRUCTION_REGISTERED + RX_BYTE_READ))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);
    sendData(RX_START);
    sendData(ID);
    sendData(RX_RWS_LENGTH);
    sendData(RX_INSTR_READ_DATA);
    sendData(RX_INSTRUCTION_REGISTERED);
    sendData(RX_BYTE_READ);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    RWS_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }

    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			RWS_Byte = readData();         // Temperature
		}
    }
	return (RWS_Byte);               // Returns the read temperature
}

int DynamixelClass::readSpeed(unsigned char ID)
{
    Checksum = (~(ID + RX_POS_LENGTH  + RX_INSTR_READ_DATA + RX_PRESENT_SPEED_L + RX_BYTE_READ_POS))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);
    sendData(RX_START);
    sendData(ID);
    sendData(RX_POS_LENGTH);
    sendData(RX_INSTR_READ_DATA);
    sendData(RX_PRESENT_SPEED_L);
    sendData(RX_BYTE_READ_POS);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    Speed_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }

    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));

			Speed_Low_Byte = readData();            // Position Bytes
			Speed_High_Byte = readData();
			Speed_Long_Byte = Speed_High_Byte << 8;
			Speed_Long_Byte = Speed_Long_Byte + Speed_Low_Byte;
		}
    }
	return (Speed_Long_Byte);     // Returns the read position
}

int DynamixelClass::readLoad(unsigned char ID)
{
    Checksum = (~(ID + RX_POS_LENGTH  + RX_INSTR_READ_DATA + RX_PRESENT_LOAD_L + RX_BYTE_READ_POS))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(RX_START);
    sendData(RX_START);
    sendData(ID);
    sendData(RX_POS_LENGTH);
    sendData(RX_INSTR_READ_DATA);
    sendData(RX_PRESENT_LOAD_L);
    sendData(RX_BYTE_READ_POS);
    sendData(Checksum);
  flush();
  switchCom(Direction_Pin,Rx_MODE);
    delayus(TX_DELAY_TIME);

    Load_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }

    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));

			Load_Low_Byte = readData();            // Position Bytes
			Load_High_Byte = readData();
			Load_Long_Byte = Load_High_Byte << 8;
			Load_Long_Byte = Load_Long_Byte + Load_Low_Byte;
		}
    }
	return (Load_Long_Byte);     // Returns the read position
}

DynamixelClass RX24F;
