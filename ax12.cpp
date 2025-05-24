/*
  ax12.cpp - ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

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

#include "ax12.h"

/******************************************************************************
 * Dynamixel communication uses Serial3
 */

namespace dynamixel_ax12 {

namespace {
uint8_t rx_buffer[kBufferSize]; // used in readResponse and getRegister
uint8_t rx_int_buffer[kBufferSize]; // also used in readResponse

// making these volatile keeps the compiler from optimizing loops of available()
// (although I'm unclear why rx_int_buffer_idx needs to be volatile)
volatile uint8_t rx_buffer_idx;
volatile uint8_t rx_int_buffer_idx;

// For sync write
uint8_t sync_write_num_servos_ = 0;
uint8_t sync_write_data_length_ = 0;
uint8_t sync_write_starting_register_;
uint8_t* sync_write_tx_buffer_;
uint8_t sync_write_tx_buffer_idx_;
}

/**
 * @brief Enable transmission on Serial3 of ATMega2560
 * 
 */
void setTX() {
  bitClear(UCSR3B, RXEN3); // disable RX
  bitSet(UCSR3B, TXEN3); // enable TX
  bitClear(UCSR3B, RXCIE3); // clear Receive Complete Interrupt
}

/**
 * @brief Enable receiving on Serial3 of ATMega2560
 * 
 */
void setRX() {
  while (bit_is_clear(UCSR3A, UDRE3)) {};
  bitClear(UCSR3B, TXEN3); // disable TX
  bitSet(UCSR3B, RXCIE3); // enable Receive Complete Interrupt
  bitSet(UCSR3B, RXEN3);  // enable RX
  rx_int_buffer_idx = 0;
  rx_buffer_idx = 0;
}

/**
 * @brief Writes a character to the Serial3/UART3 transmit buffer
 * 
 * @param data 
 */
void write(uint8_t data) {
  while (bit_is_clear(UCSR3A, UDRE3)) {}; // wait for tx buffer to be ready to receive new data
  UDR3 = data;
}

/**  */
/**
 * @brief USART3 Rx Complete Interrupt Vector - loads the received byte into 'rx_int_buffer'
 * @details From ArbotiX library for AX/RX control:
 * "We have a one-way recieve buffer, which is reset after each packet is receieved.
 *  A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps.""
 * 
 */
ISR(USART3_RX_vect){
  rx_int_buffer[(rx_int_buffer_idx++)] = UDR3;
}

namespace {
uint8_t rx_error;
}

/**
 * @brief Returns the error from the last packet read (may not relate to the last instruction sent)
 * 
 * @return uint8_t Error type
 */
uint8_t getLastError() { return rx_error; }

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
  uint8_t byte_count = 0;
  bool timeout = false;
  volatile uint8_t rx_int_buffer_idx_last = 0; // even though rx_int_buffer_idx is volatile, this must be too

  while (byte_count < length) {
    uint32_t ulCounter = 0;
    // Wait until the buffer index been incremented (by serial receive interrupt)
    while (rx_int_buffer_idx_last == rx_int_buffer_idx) {
      // But if wait too long, assume no more data coming
      if (ulCounter++ > kTimeoutCycles) {
        timeout = true;
        break; // TODO or just return false immediately?
      }
    }
    if (timeout) break;

    // Copy new byte, then test if ok
    rx_buffer[byte_count] = rx_int_buffer[rx_int_buffer_idx_last];
    if ((byte_count == 0) && (rx_buffer[0] != 0xff)) {
      // bad data, don't increment byte_count
    } else if ((byte_count == 2) && (rx_buffer[2] == 0xff)) {
      // bad data, don't increment byte_count (technically this was good, and an earlier identical byte was bad)
    } else {
      byte_count++;
    }

    rx_int_buffer_idx_last++; // assume that rx_int_buffer_idx hasn't received more than 1 byte since last time
  }

  // data plus checksum should add to 255
  uint8_t checksum = 0;
  for (uint8_t idx = 2; idx < byte_count; idx++) {
    checksum += rx_buffer[idx];
  }

  return checksum == 255;
}


/**
 * @brief Initialise UART3 on the ATMega2560
 * Set the buad rate and packet format
 * Put into receiving state
 * @param baud 
 */
void init(const uint32_t baud) {
  uint16_t ubrr = (F_CPU / (16UL * baud) - 1);
  UBRR3H = ubrr >> 8;
  UBRR3L = ubrr;

  // Set pullup on RX? Port J, bit 0
  // DDRJ |= (0 << 0);
  // PORTJ |= (1 << 0);
  // or
  bitClear(DDRJ, PJ0);
  bitSet(PORTJ, PJ0);

  // Set frame format: 8data, 1stop bit, 0 parity (default)

  // 1 stop bit (default)
  bitClear(UCSR3C, USBS3);

  // 8 data bits
  bitClear(UCSR3C, UCSZ32);
  bitSet(UCSR3C, UCSZ31);
  bitSet(UCSR3C, UCSZ30);

  // no parity
  bitClear(UCSR3C, UPM31);
  bitClear(UCSR3C, UPM30);

  // alternative UCSR3C = (0<<USBS3) | (3<<UCSZ30) | (0<<UCSZ30) ;
  setRX();
}



/**
 * @brief Ping a servo and read its status return packet
 * @details Corresponds to a PING instruction followed by parsing the response from the AX12.
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
 * @brief Write the content of a buffer out to Serial3/UART3, adding preceding 0xFF 0xFF, and trailing checksum.
 * @param buffer Containing the instruction packet, excluding checksum
 * @param length Length of buffer
 */
void writeBufferOut(const uint8_t* buffer, const uint8_t length) {
  setTX();
  write(0xFF);
  write(0xFF);
  uint8_t checksum = 0;
  for (uint8_t idx = 0; idx < length; idx++) {
    write(buffer[idx]);
    checksum += buffer[idx];
  }
  write(~checksum);
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
    rx_error = rx_buffer[4]; // 0xFF 0xFF ID LENGTH ERR P1 CHECKSUM
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
 * @param id The servo ID to write to
 * @param regstart Starting position in the control table to write to
 * @param data The data byte to write
 * @param read_response Whether to try reading a status packet. Set to false if the Status Return Level is not ALL
 */
void setRegister(const uint8_t id, const uint8_t regstart, const uint8_t data, bool read_response) {
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
    readResponse(kStatusReturnDefaultSize);
  }
}

// TODO explicitly provide the data length?
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
 * Note that multi-byte registers are stored litt-endian (low byte then high byte).
 * 
 * @param id The servo ID to write to
 * @param regstart Starting position in the control table to write to
 * @param data The 2 data bytes to write
 * @param read_response Whether to try reading a status packet. Set to false if the Status Return Level is not ALL
 */
void setRegister(const uint8_t id, const uint8_t regstart, const uint16_t data, bool read_response) {
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
    readResponse(kStatusReturnDefaultSize);
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
 */
void setStagedInstruction(const uint8_t id, const uint8_t starting_register, const uint8_t data, bool read_response) {
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
    readResponse(kStatusReturnDefaultSize);
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
 */
void setStagedInstruction(const uint8_t id, const uint8_t starting_register, const uint16_t data, bool read_response) {
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
    readResponse(kStatusReturnDefaultSize);
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

/**
 * @brief pointer to rx_int_buffer for debugging
 * @return
 */
const uint8_t* getRxIntBuffer() {
  return rx_int_buffer;
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
