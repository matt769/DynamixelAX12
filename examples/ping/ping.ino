#include <ax12.h>

namespace ax12 = dynamixel_ax12;

constexpr uint8_t kStartingId = 1;
constexpr uint8_t kNumServos = 18;

void printBuffer(const uint8_t* buffer, uint8_t length) {
  for (int i=0;i<length;i++) {
    Serial.print(buffer[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}


void setup() {
  ax12::init(1000000);
  Serial.begin(115200);
  Serial.println(F("Starting..."));
  delay(500);
}

void loop() {

    for (int servo_id = kStartingId; servo_id <= kStartingId+kNumServos; servo_id++) {
        bool res = ax12::ping(servo_id);
	Serial.print(servo_id);
	Serial.print('\t');
	Serial.print(res);
	Serial.print('\n');
  }

  delay(1000);
}

