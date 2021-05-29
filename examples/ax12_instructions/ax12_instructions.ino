#include <ax12.h>

namespace ax12 =  dynamixel_ax12;

constexpr uint8_t kNumServos = 18;

void printBuffer(uint8_t* buffer, uint8_t length) {
  for (int i=0;i<length;i++) {
    Serial.print(buffer[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}


void setup() {
  // put your setup code here, to run once:
  ax12::init(1000000);
  Serial.begin(115200);

  // Useful for debugging to be able to inspect the buffers
  uint8_t* rx_buffer = ax12::getRxBuffer();
  uint8_t* rx_int_buffer = ax12::getRxIntBuffer();


  // 1. Ping
  ax12::ping(1);
  printBuffer(rx_buffer, 6);


  int16_t current_position[18];

  // 2. Read registers (positions)
  for (int servo_id = 1; servo_id <= kNumServos; servo_id++) {
    uint16_t pos = ax12::getRegister(servo_id, RegisterPosition::AX_PRESENT_POSITION_L, 2);
    Serial.println(pos);
    printBuffer(rx_buffer, 8);
    if (pos > 1023) {
      Serial.println("Read error"); {
        while(1);
      }
    }
    uint8_t idx = servo_id-1;
    current_position[idx] = (int16_t)pos;
  }

  // 3. Write register (position)
  const uint8_t servo_id = 12;
  int16_t goal_pos = 800;
  int16_t diff = goal_pos - current_position[servo_id-1];
  const int16_t num_steps = 40;
  int16_t inc = diff / num_steps;
  for (uint8_t step = 0; step < num_steps; step++) {
    current_position[servo_id-1] += inc;
    ax12::setRegister(servo_id, RegisterPosition::AX_GOAL_POSITION_L, (uint16_t)current_position[servo_id-1], true);
    printBuffer(rx_buffer, 6);
    delay(100);
  }
  goal_pos = 200;
  diff = goal_pos - current_position[servo_id];
  inc = diff / num_steps;
  for (uint8_t step = 0; step < num_steps; step++) {
    current_position[servo_id-1] += inc;
    ax12::setRegister(servo_id, RegisterPosition::AX_GOAL_POSITION_L, (uint16_t)current_position[servo_id-1], true);
    printBuffer(rx_buffer, 6);
    delay(100);
  }
  delay(1000);

  // 4. Staged write (position)
  for (uint8_t idx = 0; idx < 2; idx++) {
    uint8_t servo_id = idx+1;
    int16_t change = current_position[idx] > 511 ? -20 : 20;
    uint16_t new_pos = current_position[idx] + change;
    ax12::setStagedInstruction(servo_id, RegisterPosition::AX_GOAL_POSITION_L, new_pos, true);
    Serial.print("Set staged instruction for servo ");
    Serial.print(servo_id);
    Serial.print(" to move to ");
    Serial.println(new_pos);
    Serial.println(getLastError());
    printBuffer(rx_buffer, 6);
  }
  // 5. And then trigger (action)
  delay(1000);
  ax12::executeStagedInstructions();
  printBuffer(rx_buffer, 6);

  // 6. Factory reset - not going to implement or test this because I dont want to have to re-set the IDs


  // 7. Sync write
  const uint8_t num_servos = 2;
  const uint8_t data_length = 2;
  const uint8_t starting_register = RegisterPosition::AX_GOAL_POSITION_L;
  const uint8_t buffer_size = num_servos * (data_length+1) + 4;
  uint8_t buffer[buffer_size];
  ax12::setupSyncWrite(num_servos, starting_register, data_length, buffer);
  uint8_t starting_servo = 11;
  for (uint8_t servo_id = starting_servo; servo_id < starting_servo + num_servos; servo_id++) {
    uint16_t new_pos = small_position_change(current_position[servo_id-1]);
    ax12::addToSyncWrite(servo_id, new_pos);
  }
  printBuffer(buffer, buffer_size);
  bool res = ax12::executeSyncWrite();
  Serial.println(res);

}



void loop() {}