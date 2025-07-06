#ifndef DYNAMIXEL_AX12_H
#define DYNAMIXEL_AX12_H

#include <Arduino.h>

namespace dynamixel_ax12 {

struct RegisterPosition {
  enum {
    /** EEPROM AREA **/
    AX_MODEL_NUMBER_L = 0,
    AX_MODEL_NUMBER_H = 1,
    AX_VERSION = 2,
    AX_ID = 3,
    AX_BAUD_RATE = 4,
    AX_RETURN_DELAY_TIME = 5,
    AX_CW_ANGLE_LIMIT_L = 6,
    AX_CW_ANGLE_LIMIT_H = 7,
    AX_CCW_ANGLE_LIMIT_L = 8,
    AX_CCW_ANGLE_LIMIT_H = 9,
    AX_SYSTEM_DATA2 = 10,
    AX_LIMIT_TEMPERATURE = 11,
    AX_DOWN_LIMIT_VOLTAGE = 12,
    AX_UP_LIMIT_VOLTAGE = 13,
    AX_MAX_TORQUE_L = 14,
    AX_MAX_TORQUE_H = 15,
    AX_RETURN_LEVEL = 16,
    AX_ALARM_LED = 17,
    AX_ALARM_SHUTDOWN = 18,
    AX_OPERATING_MODE = 19,
    AX_DOWN_CALIBRATION_L = 20,
    AX_DOWN_CALIBRATION_H = 21,
    AX_UP_CALIBRATION_L = 22,
    AX_UP_CALIBRATION_H = 23,
/** RAM AREA **/
    AX_TORQUE_ENABLE = 24,
    AX_LED = 25,
    AX_CW_COMPLIANCE_MARGIN = 26,
    AX_CCW_COMPLIANCE_MARGIN = 27,
    AX_CW_COMPLIANCE_SLOPE = 28,
    AX_CCW_COMPLIANCE_SLOPE = 29,
    AX_GOAL_POSITION_L = 30,
    AX_GOAL_POSITION_H = 31,
    AX_GOAL_SPEED_L = 32,
    AX_GOAL_SPEED_H = 33,
    AX_TORQUE_LIMIT_L = 34,
    AX_TORQUE_LIMIT_H = 35,
    AX_PRESENT_POSITION_L = 36,
    AX_PRESENT_POSITION_H = 37,
    AX_PRESENT_SPEED_L = 38,
    AX_PRESENT_SPEED_H = 39,
    AX_PRESENT_LOAD_L = 40,
    AX_PRESENT_LOAD_H = 41,
    AX_PRESENT_VOLTAGE = 42,
    AX_PRESENT_TEMPERATURE = 43,
    AX_REGISTERED_INSTRUCTION = 44,
    AX_PAUSE_TIME = 45,
    AX_MOVING = 46,
    AX_LOCK = 47,
    AX_PUNCH_L = 48,
    AX_PUNCH_H = 49
  };
};

struct StatusReturnLevel {
  enum type {
    AX_RETURN_PING_ONLY = 0,
    AX_RETURN_PING_AND_READ = 1,
    AX_RETURN_ALL = 2
  };
};

struct InstructionSet {
  enum {
    AX_PING = 1,
    AX_READ_DATA = 2,
    AX_WRITE_DATA = 3,
    AX_REG_WRITE = 4,
    AX_ACTION = 5,
//    AX_FACTORY_RESET = 6,
    AX_SYNC_WRITE = 131
  };
};

struct Error {
  enum {
    ERR_NONE = 0,
    ERR_VOLTAGE = 1,
    ERR_ANGLE_LIMIT = 2,
    ERR_OVERHEATING = 4,
    ERR_RANGE = 8,
    ERR_CHECKSUM = 16,
    ERR_OVERLOAD = 32,
    ERR_INSTRUCTION = 64
  };
};


void init(HardwareSerial* serial, uint32_t buad_rate, uint8_t direction_pin);
void setDirectionPinOutputLevelForTx(uint8_t level);

const uint8_t* getRxBuffer();
// const uint8_t* getRxIntBuffer();

int16_t getRegister(uint8_t id, uint8_t regstart, uint8_t data_length);
bool setRegister(uint8_t id, uint8_t regstart, uint8_t data, bool read_response);
bool setRegister(uint8_t id, uint8_t regstart, uint16_t data, bool read_response);

bool ping(uint8_t id);
bool setStagedInstruction(uint8_t id, uint8_t starting_register, uint8_t data, bool read_response);
bool setStagedInstruction(uint8_t id, uint8_t starting_register, uint16_t data, bool read_response);
void executeStagedInstructions();

void setupSyncWrite(uint8_t num_servos, uint8_t starting_register, uint8_t data_length, uint8_t* tx_buffer);
bool addToSyncWrite(uint8_t id, uint8_t data); // single byte version
bool addToSyncWrite(uint8_t id, uint16_t data); // double byte version
void addToSyncWrite(uint8_t id, const uint8_t* data); // for more than 2 bytes
bool executeSyncWrite();

uint8_t getLastError();

// Functions built on top of 'raw' instruction functions
void setStatusReturnLevel(uint8_t id, StatusReturnLevel::type srl);

void enableTorque(uint8_t servo_id);
void disableTorque(uint8_t servo_id);

// TODO Read registers
//  May want to change the way that getRegister works first i.e. not fixed to return int16_t and -1 on failure
//uint16_t GetModelNumber(uint8_t servo_id);
//uint8_t GetFirmwareVersion(uint8_t servo_id);
//uint8_t GetID(uint8_t servo_id);
//uint8_t GetBaudRate(uint8_t servo_id);
//uint8_t GetReturnDelayTime(uint8_t servo_id);
//uint16_t GetCWAngleLimit(uint8_t servo_id);
//uint16_t GetCCWAngleLimit(uint8_t servo_id);
//uint8_t GetTemperatureLimitMaximum(uint8_t servo_id);
//uint8_t GetMinVoltageLimit(uint8_t servo_id);
//uint8_t GetMaxVoltageLimit(uint8_t servo_id);
//uint16_t GetMaxTorque(uint8_t servo_id);
//uint8_t GetStatusReturnLevel(uint8_t servo_id);
//uint8_t GetAlarmLED(uint8_t servo_id);
//uint8_t GetShutdown(uint8_t servo_id);
//uint8_t GetTorqueEnable(uint8_t servo_id);
//uint8_t GetLEDStatus(uint8_t servo_id);
//uint8_t GetCWComplianceMargin(uint8_t servo_id);
//uint8_t GetCCWComplianceMargin(uint8_t servo_id);
//uint8_t GetCWComplianceSlope(uint8_t servo_id);
//uint8_t GetCCWComplianceSlope(uint8_t servo_id);
//uint16_t GetGoalPosition(uint8_t servo_id);
//uint16_t GetMovingSpeed(uint8_t servo_id);
//uint16_t GetTorqueLimit(uint8_t servo_id);
//uint16_t GetPresentPosition(uint8_t servo_id);
//uint16_t GetPresentSpeed(uint8_t servo_id);
//uint16_t GetPresentLoad(uint8_t servo_id);
//uint8_t GetPresentVoltage(uint8_t servo_id);
//uint8_t GetPresentTemperature(uint8_t servo_id);
//uint8_t GetRegistered(uint8_t servo_id);
//uint8_t GetMoving(uint8_t servo_id);
//uint8_t GetLock(uint8_t servo_id);
//uint16_t GetPunch(uint8_t servo_id);


void setTX();
void setRX();

// void write(uint8_t data);
void writeBufferOut(const uint8_t* buffer, uint8_t length);

bool readResponse(uint8_t length);

} // namespace

#endif // DYNAMIXEL_AX12_H
