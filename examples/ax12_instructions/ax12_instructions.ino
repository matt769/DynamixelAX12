#include <Arduino.h>
#include <ax12.h>

namespace ax12 =  dynamixel_ax12;

void printBuffer(const uint8_t* buffer, const uint8_t length) {
  for (int i=0;i<length;i++) {
    Serial.print(buffer[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}


void setup() {
  // put your setup code here, to run once:
  ax12::AX12Bus ax12bus(Serial3, 1000000, 7);
  Serial.begin(115200);

  // Useful for debugging to be able to inspect the buffers
  const uint8_t* rx_buffer = ax12bus.getRxBuffer();


  // 1. Ping
  uint8_t servo_id = 5;
  bool ping_result = ax12bus.ping(servo_id);
  Serial.print("Pinged servo id ");
  Serial.print(servo_id);
  Serial.print(". Response: ");
  Serial.println(ping_result);
  if (!ping_result) {
    Serial.println("Error. Stopping.");
    printBuffer(rx_buffer, 6);
  }
 
  // 2. Read registers (positions)
  int16_t current_position = ax12bus.getRegister(servo_id, ax12::RegisterPosition::AX_PRESENT_POSITION_L, 2);
  Serial.print("Reading position of servo id ");
  Serial.print(servo_id);
  Serial.print(". Response: ");
  Serial.println(current_position);
  if (current_position < 0) {
      Serial.println("Error. Stopping.");
      printBuffer(rx_buffer, 8);
      while(true);
  }

  // Also read the current return level
  int16_t return_level = ax12bus.getRegister(servo_id, ax12::RegisterPosition::AX_RETURN_LEVEL, 1);
  if (return_level < 0) {
      Serial.println("Error. Stopping.");
      printBuffer(rx_buffer, 8);
      while(true);
  }
  bool read_response_for_reads = return_level > 0;
  bool read_response_for_others = return_level > 1;

  // 3. Write register (position)
  int16_t change = current_position > 511 ? -20 : 20;
  uint16_t target_position = current_position + change;

  const bool set_reg_result = ax12bus.setRegister(servo_id, ax12::RegisterPosition::AX_GOAL_POSITION_L, target_position, read_response_for_others);
  Serial.print("Writing position of servo id ");
  Serial.print(servo_id);
  Serial.print(". Response: ");
  Serial.println(set_reg_result);
  if (set_reg_result) {
      current_position = target_position;
  } else {
      Serial.println("Error. Stopping.");
      printBuffer(rx_buffer, 6);
      while(true);
  }

  // 4. Staged write (position)
  int16_t change2 = current_position > 511 ? -20 : 20;
  uint16_t target_position2 = current_position + change2;
  bool set_staged_result = ax12bus.setStagedInstruction(servo_id, ax12::RegisterPosition::AX_GOAL_POSITION_L, target_position2, read_response_for_others);
  Serial.print("Set staged instruction for servo ");
  Serial.print(servo_id);
  Serial.print(" to move to ");
  Serial.println(target_position2);

  // 5. And then trigger (action)
  delay(1000);
  Serial.print("Triggering staged instructions");
  ax12bus.executeStagedInstructions();

  // 6. Factory reset - not going to implement or test this because I dont want to have to re-set the IDs
  //  since I have all my servos set into a robot currently

  // 7. Sync write
  Serial.println("Setting up synced write");
  int16_t change3 = target_position2 > 511 ? -20 : 20;
  uint16_t target_position3 = target_position2 + change3;

  const uint8_t num_servos = 1;
  const uint8_t data_length = 2;
  const uint8_t starting_register = ax12::RegisterPosition::AX_GOAL_POSITION_L;
  const uint8_t buffer_size = num_servos * (data_length+1) + 4;
  uint8_t buffer[buffer_size];
  ax12bus.setupSyncWrite(num_servos, starting_register, data_length, buffer);
  uint8_t starting_servo = 4;
  for (uint8_t servo_id = starting_servo; servo_id < starting_servo + num_servos; servo_id++) {
    ax12bus.addToSyncWrite(servo_id, target_position3);
  }
  Serial.print("Executing synced write");
  bool res = ax12bus.executeSyncWrite();

}



void loop() {}

