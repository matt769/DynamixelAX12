#include <ax12.h>

namespace ax12 =  dynamixel_ax12;

constexpr uint8_t kStartingId = 5;
constexpr uint8_t kNumServos = 1;
int16_t current_position[18]; // data for servo N held at index N-1 i.e. expect ervo numbering to start at 1

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
  const uint8_t* const rx_buffer = ax12::getRxBuffer();
  const uint8_t* const rx_int_buffer = ax12::getRxIntBuffer();


  // 1. Ping
  Serial.println("*** 1: PING ***");
  for (int servo_id = kStartingId; servo_id < kStartingId+kNumServos; servo_id++) {
    const bool res = ax12::ping(servo_id);
    Serial.print(servo_id);
    Serial.print('\t');
    Serial.print(res);
    Serial.print('\n');
//    printBuffer(rx_buffer, 6);
    if (!res) {
      Serial.println("No response");
        while(1);
    }
  }
  delay(1000);



  // 2. Read registers (positions)
  Serial.println("*** 2: Read register ***");
  for (int servo_id = kStartingId; servo_id < kStartingId+kNumServos; servo_id++) {
    const uint16_t pos = ax12::getRegister(servo_id, ax12::RegisterPosition::AX_PRESENT_POSITION_L, 2);
    Serial.println(pos);
//    printBuffer(rx_buffer, 8);
    if (pos > 1023) {
      Serial.println("Read error"); {
        while(1);
      }
    }
    const uint8_t idx = servo_id-1;
    current_position[idx] = (int16_t)pos;
  }
  delay(1000);


  // 3. Write register (position)
  Serial.println("*** 3: Write register ***");
  for (int servo_id = kStartingId; servo_id < kStartingId+kNumServos; servo_id++) {
    const uint8_t idx = servo_id - 1;
    const int16_t change = current_position[idx] > 511 ? -40 : 40;
    const uint16_t goal_pos = current_position[idx] + change;
    ax12::setRegister(servo_id, ax12::RegisterPosition::AX_GOAL_POSITION_L, goal_pos, true);
    current_position[idx] = goal_pos;
//    printBuffer(rx_buffer, 6);
    delay(100);
  }
  delay(1000);


  // 4. Staged write (position)
  Serial.println("*** 4: Staged write ***");
  for (int servo_id = kStartingId; servo_id < kStartingId+kNumServos; servo_id++) {
    const uint8_t idx = servo_id - 1;
    const int16_t change = current_position[idx] > 511 ? -40 : 40;
    const uint16_t goal_pos = current_position[idx] + change;
    ax12::setStagedInstruction(servo_id, ax12::RegisterPosition::AX_GOAL_POSITION_L, goal_pos, true);
    current_position[idx] = goal_pos;
    Serial.print("Set staged instruction for servo ");
    Serial.print(servo_id);
    Serial.print(" to move to ");
    Serial.println(goal_pos);
    // Serial.println(ax12::getLastError());
//    printBuffer(rx_buffer, 6);
  }
  
  // 5. And then trigger (action)
  delay(1000);
  Serial.println("*** 5: Trigger staged action ***");
  ax12::executeStagedInstructions();
  // printBuffer(rx_buffer, 6);
  delay(1000);

  // 6. Factory reset - not going to implement or test this because I dont want to have to re-set the IDs

  // 7. Sync write
    Serial.println("*** 2: Sync write ***");
  const uint8_t data_length = 2;
  const uint8_t starting_register = ax12::RegisterPosition::AX_GOAL_POSITION_L;
  const uint8_t buffer_size = kNumServos * (data_length+1) + 4;
  uint8_t buffer[buffer_size];
  ax12::setupSyncWrite(kNumServos, starting_register, data_length, buffer);

  for (uint8_t servo_id = kStartingId; servo_id < kStartingId+kNumServos; servo_id++) {
    const uint8_t idx = servo_id - 1;
    const int16_t change = current_position[idx] > 511 ? -40 : 40;
    const uint16_t goal_pos = current_position[idx] + change;
    ax12::addToSyncWrite(servo_id, goal_pos);
    current_position[idx] = goal_pos;
  }
//  printBuffer(buffer, buffer_size);
  bool res = ax12::executeSyncWrite();
  Serial.println(res);

}



void loop() {}
