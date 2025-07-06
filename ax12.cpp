#include "ax12.h"

namespace dynamixel_ax12 {

constexpr uint8_t kBufferSize = 32;
constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint32_t kTimeoutCycles = 1500; // num cycles to wait for each byte in response packet before timing out
constexpr uint8_t kStatusReturnDefaultSize = 6;

namespace {
uint8_t rx_buffer[kBufferSize]; // used in readResponse and getRegister

// For sync write
uint8_t sync_write_num_servos_ = 0;
uint8_t sync_write_data_length_ = 0;
uint8_t sync_write_starting_register_;
uint8_t *sync_write_tx_buffer_;
uint8_t sync_write_tx_buffer_idx_;

uint8_t rx_error_;

HardwareSerial* serial_;
uint8_t direction_pin_;
uint8_t tx_level_ = LOW;
}

/**
 * @brief Enable the tx line / disable rx line
 * 
 */
void setTX() {
  digitalWrite(direction_pin_, tx_level_);
}

/**
 * @brief Enable the rx line / disable tx line
 * 
 */
void setRX() {
  serial_->flush(); //ensure any transmissions have been fully sent
  digitalWrite(direction_pin_, !tx_level_);
}


/**
 * @brief Returns the error from the last packet read (may not relate to the last instruction sent)
 * 
 * @return uint8_t Error type
 */
uint8_t getLastError() { return rx_error_; }

/**
 * @brief Reads a status response packet from an AX12
 * @details May be called IMMEDIATELY after issuing an instruction.
 * Most status return packets that I have examined have 1 or (usually) 2 leading bytes before the 0xFF 0xFF
 * I assume this is picked up from the outgoing instruction transmission (since tx is physically tied to rx)
 *  although the spurious received bytes don't appear to directly correspond to those received
 *  e.g.
 *  SENT:       255	255	1	2	1	251 (ping servo 1)
 *  RECEIVED:   64	255	255	255	1	2	0	252
 * e.g.
 * SENT:        255	255	2	2	1	250 (ping servo 2)
 * RECEIVED:    255	255	255	2	2	0	251
 *
 * It appears to be quite common to have 255 as the 2nd spurious byte, which we need to NOT consider as part of the
 *  leading 2 255s
 * 
 * @param length Length of expected status packet. Normally 6, but may be more following a READ instruction.
 * @return true if packet of expected length with validated checksum was received
 * @return false otherwise (timeout or failed checksum)
 */
bool readResponse(const uint8_t length) {
  uint8_t rx_idx = 0;
  uint32_t start_time_us = micros();
  while (rx_idx < length && micros() - start_time_us < kTimeoutCycles) {
    if (serial_->available()) {
      rx_buffer[rx_idx++] = serial_->read();
    }
  }
  if (rx_idx < length) {
    return false; // timeout
  }

  // data plus checksum byte (final byte in status return packet) should add to 255
  uint8_t total = 0;
  for (uint8_t idx = 2; idx < length; idx++) {
    total += rx_buffer[idx];
  }
  
  const bool checksum_ok = total == 255;
  const bool is_error = rx_buffer[4] > 0;

  return checksum_ok && !is_error;
}


/**
 * @brief Initialise controller setup for hardware half-duplex communication
 * - initialises the chosen UART interface
 * - sets up direction control pin
 * - sets the transmission direction to receive.
 * 
 * @param serial the serial/UART interface to use
 * @param baud_rate
 * @param direction_pin The pin number that is wired to the direction control of the hardware half-duplex setup
 */
void init(HardwareSerial* serial, const uint32_t baud_rate, const uint8_t direction_pin) {
  serial_ = serial;
  direction_pin_ = direction_pin;

  serial_->begin(baud_rate);
  pinMode(direction_pin_, OUTPUT);
  setRX();
}

void setDirectionPinOutputLevelForTx(uint8_t level) {
  tx_level_ = level;
}


/**
 * @brief Ping a servo and read its status return packet
 * @details Corresponds to a PING instruction followed by parsing the response from the AX12.
 * The response will always be sent regardless of the status return level of the target.
 * The READ instruction has the following format
 * Instruction Length 	  Instruction ID
 * 0x02 	                0x01
 * 
 * @param id The servo to ping
 * @return bool the result of readResponse
 */
bool ping(const uint8_t id) {
  const uint8_t packet_length = 2; // Byte size of the Instruction (1), Parameter (0) and Checksum (1) fields.
  const uint8_t buffer_size = 3;
  uint8_t tx_buffer[buffer_size];

  tx_buffer[0] = id;
  tx_buffer[1] = packet_length;
  tx_buffer[2] = InstructionSet::AX_PING;

  writeBufferOut(tx_buffer, buffer_size);

  return readResponse(kStatusReturnDefaultSize);
}

/**
 * @brief Write the contents of a buffer out, adding preceding 0xFF 0xFF, and trailing checksum.
 * @param buffer Containing the instruction packet, excluding checksum
 * @param length Length of buffer
 */
void writeBufferOut(const uint8_t* buffer, const uint8_t length) {
  setTX();

  serial_->write(0xFF);
  serial_->write(0xFF);
  uint8_t checksum = 0;
  for (uint8_t idx = 0; idx < length; idx++) {
    serial_->write(buffer[idx]);
    checksum += buffer[idx];
  }
  serial_->write(~checksum);

  serial_->flush(); // wait until everything has actually been transmitted
  setRX();
}

/**
 * @brief Return the value of a register on the AX12
 * @details Corresponds to a READ instruction followed by parsing the response from the AX12.
 * The READ instruction has the following format
 * Instruction Length 	  Instruction ID 	  P1 	                P2
 * 0x04 	                0x02 	            Starting Address 	  Length of Data
 * 
 * where P1 and P2 are the parameters of this instruction,
 * and this structure is always preceeded by 0xFF 0xFF in the serial packet.
 * 
 * @param id The servo ID to read from
 * @param regstart Starting position in the control table to read from 
 * @param data_length Register length in bytes
 * @return int16_t the read value, or -1 if unsuccessful
 */
int16_t getRegister(const uint8_t id, const uint8_t regstart, const uint8_t data_length) {
  // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
  const uint8_t packet_length = 4; // Byte size of the Instruction (1), Parameter (2) and Checksum (1) fields.
  const uint8_t buffer_size = 5;
  uint8_t tx_buffer[buffer_size];
  tx_buffer[0] = id;
  tx_buffer[1] = packet_length;
  tx_buffer[2] = InstructionSet::AX_READ_DATA;
  tx_buffer[3] = regstart;
  tx_buffer[4] = data_length;

  writeBufferOut(tx_buffer, buffer_size);

  if (readResponse(data_length + kStatusReturnDefaultSize) > 0) {
    rx_error_ = rx_buffer[4]; // 0xFF 0xFF ID LENGTH ERR P1 CHECKSUM
    if (data_length == 1) {
      return rx_buffer[5];
    } else {
      return rx_buffer[5] + (rx_buffer[6] << 8);
    }

  } else {
    return -1;
  }
}

/**
 * @brief Set the value of a single-byte register on the AX12
 * @details Corresponds to a WRITE instruction.
 * The WRITE instruction has the following general format
 * Instruction Length 	  Instruction ID 	  P1 	                P2          P3          PN+1
 * N + 3 	                0x02 	            Starting Address 	  1st byte    2nd byte    ...etc
 * 
 * where P1, P2 ... PN+1 are the parameters of this instruction. In this function there is only P1, P2
 * In theory multiple (consecutive) registers can be written at once (not supported by this function)
 * This structure is always preceeded by 0xFF 0xFF in the serial packet.
 *
 * Whether or not the target sends a response depends on its status return level. If this command will trigger a response, you should read it (even if you don't use it).
 *
 * @param id The servo ID to write to
 * @param regstart Starting position in the control table to write to
 * @param data The data byte to write
 * @param read_response Whether to try reading a status packet. Set to false if the Status Return Level is not ALL
 * @return true if no response requested, otherwise true if a response was received and there were no errors reported
 */
bool setRegister(const uint8_t id, const uint8_t regstart, const uint8_t data, bool read_response) {
  const uint8_t packet_length = 4; // Byte size of the Instruction (1), Parameter (2) and Checksum (1) fields.
  const uint8_t buffer_size = 5;
  uint8_t tx_buffer[buffer_size];
  tx_buffer[0] = id;
  tx_buffer[1] = packet_length;
  tx_buffer[2] = InstructionSet::AX_WRITE_DATA;
  tx_buffer[3] = regstart;
  tx_buffer[4] = data;

  writeBufferOut(tx_buffer, buffer_size);

  if (read_response) {
    return readResponse(kStatusReturnDefaultSize);
  } else {
    return true;
  }
}

/**
 * @brief Set the value of a double-byte register on the AX12
 * @details Corresponds to a WRITE instruction.
 * The WRITE instruction has the following general format
 * Instruction Length 	  Instruction ID 	  P1 	                P2          P3          PN+1
 * N + 3 	                0x03 	            Starting Address 	  1st byte    2nd byte    ...etc
 * 
 * where P1, P2 ... PN+1 are the parameters of this instruction. In this function there is only P1, P2, P3
 * In theory multiple (consecutive) registers can be written at once (not supported by this function)
 * This structure is always preceeded by 0xFF 0xFF in the serial packet.
 * 
 * Note that multi-byte registers are stored little-endian (low byte then high byte).
 * 
 * @param id The servo ID to write to
 * @param regstart Starting position in the control table to write to
 * @param data The 2 data bytes to write
 * @param read_response Whether to try reading a status packet. Set to false if the Status Return Level is not ALL
 * @return true if no response requested, otherwise true if a response was received and there were no errors reported
 */
bool setRegister(const uint8_t id, const uint8_t regstart, const uint16_t data, bool read_response) {
  const uint8_t packet_length = 5; // Byte size of the Instruction (1), Parameter (3) and Checksum (1) fields.
  const uint8_t buffer_size = 6;
  uint8_t tx_buffer[buffer_size];
  tx_buffer[0] = id;
  tx_buffer[1] = packet_length;
  tx_buffer[2] = InstructionSet::AX_WRITE_DATA;
  tx_buffer[3] = regstart;
  tx_buffer[4] = data & 0xff;
  tx_buffer[5] = (data & 0xff00) >> 8;

  writeBufferOut(tx_buffer, buffer_size);

  if (read_response) {
    return readResponse(kStatusReturnDefaultSize);
  } else {
    return true;
  }
}

/**
 * @brief Stage a single-byte register change on to the AX12, to be activated separately
 * @details Corresponds to a REG_WRITE instruction.
 * The WRITE instruction has the following general format
 * Instruction Length 	  Instruction ID 	  P1 	                P2          P3          PN+1
 * N + 3 	                0x04 	            Starting Address 	  1st byte    2nd byte    ...etc
 * 
 * where P1, P2 ... PN+1 are the parameters of this instruction. In this function there is only P1, P2
 * In theory multiple (consecutive) registers can be written at once (not supported by this function)
 * This structure is always preceeded by 0xFF 0xFF in the serial packet.
 * @param id The servo to write to
 * @param starting_register Starting position in the control table to write to
 * @param data The data byte to write
 * @param read_response Whether to try reading a status packet. Set to false if the Status Return Level is not ALL
 * @return true if no response requested, otherwise true if a response was received and there were no errors reported
 */
bool setStagedInstruction(const uint8_t id, const uint8_t starting_register, const uint8_t data, bool read_response) {
  const uint8_t packet_length = 4; // Byte size of the Instruction (1), Parameter (2) and Checksum (1) fields.
  const uint8_t buffer_size = 5;
  uint8_t tx_buffer[buffer_size];
  tx_buffer[0] = id;
  tx_buffer[1] = packet_length;
  tx_buffer[2] = InstructionSet::AX_REG_WRITE;
  tx_buffer[3] = starting_register;
  tx_buffer[4] = data;

  writeBufferOut(tx_buffer, buffer_size);

  if (read_response) {
    return readResponse(kStatusReturnDefaultSize);
  } else {
    return true;
  }
}


/**
 * @brief Stage a double-byte register change on to the AX12, to be activated separately
 * @details Corresponds to a REG_WRITE instruction.
 * The REG_WRITE instruction has the following general format
 * Instruction Length 	  Instruction ID 	  P1 	                P2          P3          PN+1
 * N + 3 	                0x04 	            Starting Address 	  1st byte    2nd byte    ...etc
 * 
 * where P1, P2 ... PN+1 are the parameters of this instruction. In this function there is only P1, P2, P3
 * In theory multiple (consecutive) registers can be written at once (not supported by this function)
 * This structure is always preceeded by 0xFF 0xFF in the serial packet.
 * 
 * This instruction is identical to the WRITE instruction, but with a different Instruction ID.
 * 
 * @param id The servo to write to
 * @param starting_register Starting position in the control table to write to
 * @param data The 2 data bytes to write
 * @param read_response Whether to try reading a status packet. Set to false if the Status Return Level is not ALL
 * @return true if no response requested, otherwise true if a response was received and there were no errors reported
 */
bool setStagedInstruction(const uint8_t id, const uint8_t starting_register, const uint16_t data, bool read_response) {
  const uint8_t packet_length = 5; // Byte size of the Instruction (1), Parameter (3) and Checksum (1) fields.
  const uint8_t buffer_size = 6;
  uint8_t tx_buffer[buffer_size];
  tx_buffer[0] = id;
  tx_buffer[1] = packet_length;
  tx_buffer[2] = InstructionSet::AX_REG_WRITE;
  tx_buffer[3] = starting_register;
  tx_buffer[4] = data & 0xff;
  tx_buffer[5] = (data & 0xff00) >> 8;

  writeBufferOut(tx_buffer, buffer_size);

  if (read_response) {
    return readResponse(kStatusReturnDefaultSize);
  } else {
    return true;
  }
}

/**
 * @brief Apply all staged writes following setStagedInstruction
 * @details Corresponds to an ACTION instruction.
 * The ACTION instruction has the following format
 * Instruction Length 	  Instruction ID
 * 2 	                    0x05 	        
 *
 * This structure is always preceeded by 0xFF 0xFF in the serial packet.
 * 
 * Note that this uses the broadcast ID to address all servos, so there is no status return packet.
 * 
 */
void executeStagedInstructions() {
  const uint8_t packet_length = 2; // Byte size of the Instruction (1), Parameter (0) and Checksum (1) fields.
  const uint8_t buffer_size = 3;
  uint8_t tx_buffer[buffer_size];
  tx_buffer[0] = kBroadcastId;
  tx_buffer[1] = packet_length;
  tx_buffer[2] = InstructionSet::AX_ACTION;

  writeBufferOut(tx_buffer, buffer_size);
}

/**
 * @brief Set up for a SYNC_WRITE instruction. Must be called before addToSyncWrite or executeSyncWrite.
 * @details The buffer to hold all the data to be transmitted must be externally provided. It must be (at least)
 *  S x (N+1) + 5 bytes long, where S is the number of servos that will be added, an N is the number of bytes that will be
 *  written to each one.
 *  E.g. Writing goal position (2 bytes) to 7 servos requires a buffer of 7*(2+1) + 5 = 26;
 *  Maximum size is 140 (AX servo rx buffer is 143 bytes).
 *
 * This should be followed with addToSyncWrite() calls to set the servo specific data values,
 *  and then executeSyncWrite() to send the instruction to all the (relevant) servos.
 *
 *
 * @param num_servos number of servos that will be added to in this instruction
 * @param starting_register starting register address from which to write data
 * @param data_length length (bytes) of data to be written
 * @param tx_buffer a buffer of sufficient length
 */
void setupSyncWrite(const uint8_t num_servos, const uint8_t starting_register, const uint8_t data_length, uint8_t* tx_buffer) {
  sync_write_num_servos_ = num_servos;
  sync_write_starting_register_ = starting_register;
  sync_write_data_length_ = data_length;
  sync_write_tx_buffer_ = tx_buffer;
  sync_write_tx_buffer_idx_ = 0;

  sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = kBroadcastId;
  sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = num_servos * (data_length + 1) + 4;
  sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = InstructionSet::AX_SYNC_WRITE;
  sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = starting_register;
  sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = data_length;
}

/**
 * @brief Set up for a single byte (per servo) SYNC_WRITE instruction. setupSyncWrite must be called first.
 * @param id
 * @param data
 */
bool addToSyncWrite(const uint8_t id, const uint8_t data) {
  if (sync_write_data_length_ == 1) {
    sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = id;
    sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = data;
    return true;
  }
  else {
    // unexpected data length based on information provided to setupSyncWrite
    return false;
  }
}

/**
 * @brief Set up for a double byte (per servo) SYNC_WRITE instruction. setupSyncWrite must be called first.
 * @param id
 * @param data
 */
bool addToSyncWrite(const uint8_t id, const uint16_t data) {
  if (sync_write_data_length_ == 2) {
    sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = id;
    sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = data & 0xff;
    sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = (data & 0xff00) >> 8;
    return true;
  }
  else {
    // unexpected data length based on information provided to setupSyncWrite
    return false;
  }
}

/**
 * @brief Set up for a multi-byte (per servo) SYNC_WRITE instruction. setupSyncWrite must be called first.
 * @param id
 * @param data
 */
void addToSyncWrite(const uint8_t id, const uint8_t* data) {
  sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = id;
  for (uint8_t i = 0; i < sync_write_data_length_; i++) {
    sync_write_tx_buffer_[sync_write_tx_buffer_idx_++] = data[i];
  }
}

/**
 * @brief Corresponds to a SYNC_WRITE instruction. Must hav been set up first with setupSyncWrite and addToSyncWrite.
 * @return true if the total bytes added to the buffer by addToSyncWrite calls matches the number of servos and
 *  data length values declared in setupSyncWrite.
 * @return false if not (and it will not attempt to transmit the instruction).
 */
bool executeSyncWrite() {
  if (sync_write_tx_buffer_idx_ != sync_write_tx_buffer_[1] + 1) {
    return false;
  }
  else {
    writeBufferOut(sync_write_tx_buffer_, sync_write_tx_buffer_[1] + 1);
    return true;
  }
}




/**
 * @brief pointer to rx_buffer for debugging
 * @return
 */
const uint8_t* getRxBuffer() {
  return rx_buffer;
}

void setStatusReturnLevel(const uint8_t id, const StatusReturnLevel::type srl) {
  setRegister(id, RegisterPosition::AX_RETURN_LEVEL, static_cast<uint8_t>(srl), false);
}

void enableTorque() {
  setRegister(kBroadcastId, RegisterPosition::AX_TORQUE_ENABLE, static_cast<uint8_t>(1), false);
}

void disableTorque() {
  setRegister(kBroadcastId, RegisterPosition::AX_TORQUE_ENABLE, static_cast<uint8_t>(0), false);
}

}
