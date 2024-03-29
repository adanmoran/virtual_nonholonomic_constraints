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

#ifndef DynamixelRX24F_h
#define DynamixelRX24F_h

	// EEPROM AREA  ///////////////////////////////////////////////////////////
#define RX_MODEL_NUMBER_L           0
#define RX_MODEL_NUMBER_H           1
#define RX_FIRMWARE_VERSION         2
#define RX_ID                       3
#define RX_BAUD_RATE                4
#define RX_RETURN_DELAY_TIME        5
#define RX_CW_ANGLE_LIMIT_L         6
#define RX_CW_ANGLE_LIMIT_H         7
#define RX_CCW_ANGLE_LIMIT_L        8
#define RX_CCW_ANGLE_LIMIT_H        9
//#define AX_SYSTEM_DATA2             10
#define RX_LIMIT_TEMPERATURE        11
#define RX_LOW_LIMIT_VOLTAGE        12
#define RX_HIGH_LIMIT_VOLTAGE       13
#define RX_MAX_TORQUE_L             14
#define RX_MAX_TORQUE_H             15
#define RX_STATUS_RETURN_LEVEL      16
#define RX_ALARM_LED                17
#define RX_ALARM_SHUTDOWN           18
//#define AX_OPERATING_MODE           19
//#define AX_DOWN_CALIBRATION_L       20
//#define AX_DOWN_CALIBRATION_H       21
//#define AX_UP_CALIBRATION_L         22
//#define AX_UP_CALIBRATION_H         23

	// RAM AREA  //////////////////////////////////////////////////////////////
#define RX_TORQUE_ENABLE            24
#define RX_LED                      25
#define RX_CW_COMPLIANCE_MARGIN     26
#define RX_CCW_COMPLIANCE_MARGIN    27
#define RX_CW_COMPLIANCE_SLOPE      28
#define RX_CCW_COMPLIANCE_SLOPE     29
#define RX_GOAL_POSITION_L          30
#define RX_GOAL_POSITION_H          31
#define RX_GOAL_SPEED_L             32
#define RX_GOAL_SPEED_H             33
#define RX_TORQUE_LIMIT_L           34
#define RX_TORQUE_LIMIT_H           35
#define RX_PRESENT_POSITION_L       36
#define RX_PRESENT_POSITION_H       37
#define RX_PRESENT_SPEED_L          38
#define RX_PRESENT_SPEED_H          39
#define RX_PRESENT_LOAD_L           40
#define RX_PRESENT_LOAD_H           41
#define RX_PRESENT_VOLTAGE          42
#define RX_PRESENT_TEMPERATURE      43
#define RX_INSTRUCTION_REGISTERED   44
//#define AX_PAUSE_TIME               45
#define RX_MOVING                   46
#define RX_LOCK                     47
#define RX_PUNCH_L                  48
#define RX_PUNCH_H                  49

    // Status Return Levels ///////////////////////////////////////////////////////////////
#define RX_RETURN_NONE              0
#define RX_RETURN_READ              1
#define RX_RETURN_ALL               2

    // Instruction Set ///////////////////////////////////////////////////////////////
#define RX_INSTR_PING               1
#define RX_INSTR_READ_DATA          2
#define RX_INSTR_WRITE_DATA         3
#define RX_INSTR_REG_WRITE          4
#define RX_INSTR_ACTION             5
#define RX_INSTR_RESET              6
#define RX_INSTR_SYNC_WRITE         131

	// Specials ///////////////////////////////////////////////////////////////
#define OFF                         0
#define ON                          1
#define LEFT						            0
#define RIGHT                       1
#define RX_BYTE_READ                1
#define RX_BYTE_READ_POS            2
#define RX_RESET_LENGTH				      2
#define RX_ACTION_LENGTH			      2
#define RX_ID_LENGTH                4
#define RX_LR_LENGTH                4
#define RX_SRL_LENGTH               4
#define RX_RDT_LENGTH               4
#define RX_LEDALARM_LENGTH          4
#define RX_SALARM_LENGTH            4
#define RX_TL_LENGTH                4
#define RX_VL_LENGTH                6
#define RX_CM_LENGTH                6
#define RX_CS_LENGTH                6
#define RX_CCW_CW_LENGTH            8
#define RX_BD_LENGTH                4
#define RX_TEM_LENGTH               4
#define RX_MOVING_LENGTH            4
#define RX_RWS_LENGTH               4
#define RX_VOLT_LENGTH              4
#define RX_LED_LENGTH               4
#define RX_TORQUE_LENGTH            4
#define RX_POS_LENGTH               4
#define RX_GOAL_LENGTH              5
#define RX_MT_LENGTH                5
#define RX_PUNCH_LENGTH             5
#define RX_SPEED_LENGTH             5
#define RX_GOAL_SP_LENGTH           7
#define RX_ACTION_CHECKSUM			    250
#define BROADCAST_ID                254
#define RX_START                    255
#define RX_CCW_AL_L                 255
#define RX_CCW_AL_H                 3
#define TIME_OUT                    10         // Este parametro depende de la velocidad de transmision
#define TX_DELAY_TIME				        0        // Este parametro depende de la velocidad de transmision - pero pueden ser cambiados para mayor velocidad.
#define Tx_MODE                     1
#define Rx_MODE                     0
#define LOCK                        1

#include <inttypes.h>

class DynamixelClass {
private:

	unsigned char Checksum;
	unsigned char Direction_Pin;
	unsigned char Time_Counter;
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

public:

	void begin(long baud, unsigned char directionPin);
	void begin(long baud);
	void end(void);

	int reset(unsigned char ID);
	int ping(unsigned char ID);

	int setID(unsigned char ID, unsigned char newID);
	int setBD(unsigned char ID, long baud);

	int move(unsigned char ID, int Position);
	int moveSpeed(unsigned char ID, int Position, int Speed);
	int setEndless(unsigned char ID,bool Status);
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
	int readSpeed(unsigned char ID);
	int readLoad(unsigned char ID);

	int torqueStatus(unsigned char ID, bool Status);
	int ledStatus(unsigned char ID, bool Status);
};

extern DynamixelClass RX24F;

#endif
