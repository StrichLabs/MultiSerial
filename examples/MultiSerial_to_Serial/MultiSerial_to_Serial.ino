// This code passes data from the defined MultiSerial shield's port and
// passes it through to the Arduino's main serial port, bidirectionally.

// TODO: Making this interrupt driven would make it a lot more efficient.

#include <Wire.h>
#include <MultiSerial.h>

#define BOARD_ADDR 0x4D
#define BOARD_PORT 0

#define ARDUINO_BAUD_RATE     9600
#define MULTISERIAL_BAUD_RATE 9600

MultiSerial msTest = MultiSerial(BOARD_ADDR, BOARD_PORT);
void setup() {
  Serial.begin(ARDUINO_BAUD_RATE);
  msTest.begin(MULTISERIAL_BAUD_RATE);
  msTest.store('U');
  if(msTest.retrieve() != 'U') {
    Serial.print("Unable to communicate with MultiSerial Shield port ");
    Serial.print(BOARD_PORT, DEC);
    Serial.print(" at address 0x");
    Serial.println(BOARD_ADDR, HEX);
    while(1)
      delay(10000);
  }
}

void loop() {
  if(msTest.available() > 0) {
    Serial.write(msTest.read());
  }
  if(Serial.available() > 0)
    msTest.write(Serial.read());
  
  delay(5);
}
