// This code tests any MultiSerial shields which are attached.
// For proper test results, connect loopback plugs to the GPIO and RS232
// connectors, and select 3.3V or RS232 levels.

#include <Wire.h>
#include <MultiSerial.h>

#define TEST_REPS 32

volatile byte int_counter;

void int_dbg(void) {
  int_counter++;
}

void debugPrintNumber(byte number) {
  Serial.print("0x");
  Serial.print(number, HEX);
  Serial.print(" b");
  Serial.println(number, BIN);
}

void showTestResults(byte numFails, byte totalTests) {
  if(numFails > 0) {
      Serial.print("FAIL: ");
      Serial.print(numFails, DEC);
      Serial.print(" out of ");
      Serial.print(totalTests, DEC);
      Serial.println(" tests failed.");
  } else {
    Serial.println("PASS!");
  }
}

void printSerialHeader(byte chan) {
  Serial.print("SERI");
  Serial.print(chan, DEC);
  Serial.print(": ");
}

void printFifoHeader(byte chan) {
  Serial.print("FIFO");
  Serial.print(chan, DEC);
  Serial.print(": ");
}

void printControllerHeader(byte chan) {
  Serial.print("CTLR");
  Serial.print(chan, DEC);
  Serial.print(": ");
}

void printGpioHeader(byte pin) {
  Serial.print("GPIO");
  Serial.print(pin, DEC);
  Serial.print(": ");
}

byte testController(MultiSerial msTest, byte chan) {
  byte rep, numFails = 0;
  
  // Write a byte to the SPR (scratchpad) register then read it to verify controller comms
  printControllerHeader(chan);
  Serial.print("Testing...");
  for(rep=0; rep < TEST_REPS; rep++) {
    byte testByte;
    Serial.print(".");
    testByte = random(0, 255);
    msTest.store(testByte);
    if(msTest.retrieve() != testByte) numFails++;
  }
  
  showTestResults(numFails, TEST_REPS);
  return numFails; 
}

byte testSerialLoopback(MultiSerial msTest, byte chan) {
  byte rep, txByte, rxByte, numFails = 0;
  
  // Write then read bytes through each serial port to verify serial comms
  printSerialHeader(chan);
  Serial.print("Testing...");
  for(rep=0; rep < TEST_REPS; rep++) {
    Serial.print(".");
    txByte = 'A' + rep;
    msTest.write(txByte);
    delay(15);
    rxByte = msTest.read();
    if(txByte != rxByte) {
//      if(chanFail == 0) {
//        printSerHeader(chan);
//        Serial.println("FAIL");
//        printSerHeader(chan);
        Serial.print("Expected ");
        debugPrintNumber(txByte);
        Serial.print(", got ");
        debugPrintNumber(rxByte);
        Serial.println("");
//      }
      numFails++;
    }
  }
  
  showTestResults(numFails, TEST_REPS);
  
  return numFails;
}

byte testSerialLoopbackFifo(MultiSerial msTest, byte chan) {
  byte rep, txByte, rxByte, numFails = 0;
  
  // Write then read bytes through each serial port to verify serial comms, using the FIFO this time
  printFifoHeader(chan);
  byte sentData[TEST_REPS/2];
  Serial.print("Testing...");
  for(rep=0; rep < TEST_REPS/2; rep++) {
    Serial.print(".");
    txByte = 'A' + rep;
    msTest.write(txByte);
    sentData[rep] = txByte;
    delay(15);
  }
  
  byte bytesAvailable = msTest.available();
  if(bytesAvailable != TEST_REPS/2) {
    numFails++;
  }
  
  for(rep=0; rep < TEST_REPS/2; rep++) {
    rxByte = msTest.read();
    if(rxByte != sentData[rep]) {
      numFails++;
    }
    Serial.print(".");
  }
  
  showTestResults(numFails, TEST_REPS+1);
  
  return numFails;
}

byte testGpio(MultiSerial msTest) {
  printGpioHeader(0);
  Serial.print("Testing...");
  byte pin, baseOutputPin, numFails = 0;
  // first time through is even output, odd input GPIO test, then it's odd output, even input
  for(baseOutputPin=0; baseOutputPin < 2; baseOutputPin++) {
    delay(1000);
    for(pin=baseOutputPin; pin<8; pin+=2) {
      byte outputPin = pin;
      byte inputPin = pin + 1;
      if(baseOutputPin == 1) inputPin = pin - 1;
      
      msTest.pinMode(inputPin, INPUT);
      msTest.pinMode(outputPin, OUTPUT);
            
      Serial.print(".");
      msTest.digitalWrite(outputPin, HIGH);
      if(msTest.digitalRead(inputPin) != HIGH) {
        Serial.print("Input pin ");
        Serial.print(inputPin, DEC);
        Serial.println(" stuck low?");
        numFails++;
      }
      Serial.print(".");
      msTest.digitalWrite(outputPin, LOW);
      if(msTest.digitalRead(inputPin) != LOW) {
        Serial.print("Input pin ");
        Serial.print(inputPin, DEC);
        Serial.println(" stuck high?");
        numFails++;
      }
    }
  }
  
  if(numFails > 0) {
    Serial.print("FAIL: ");
    Serial.print(numFails, DEC);
    Serial.println(" GPIO tests failed.");
  } else {
    Serial.println("PASS!");
  }
  
  return numFails;
}

void setup() {
  byte boardNbr;
  byte boardAddresses[3];
  boardAddresses[0] = 0x4d;
  boardAddresses[1] = 0x4c;
  boardAddresses[2] = 0x49;
  boardAddresses[3] = 0x48;
  
  int_counter = 0;
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);
  attachInterrupt(0, int_dbg, FALLING);
  
  Serial.begin(9600);
  Serial.println("\n");
  for(boardNbr = 0; boardNbr < 4; boardNbr++) {
    byte chan;
    byte boardFails = 0;
    byte boardAddr = boardAddresses[boardNbr];
    
    // check if there's even a board here before we try to test it
    MultiSerial msTest = MultiSerial(boardAddr, 0);
    msTest.begin(9600);
    msTest.store(0xAA);
    if(msTest.retrieve() != 0xAA) {
      Serial.print("No board found at address 0x");
      Serial.print(boardAddr, HEX);
      Serial.println(", skipping.");
      continue;
    }
    
    // looks like there's a board here, so test it!
    Serial.print("Starting test of board at address 0x");
    Serial.print(boardAddr, HEX);
    Serial.println("...");
    for(chan=0; chan <= 1; chan++) {
      MultiSerial msTest = MultiSerial(boardAddr, chan);
      msTest.begin(9600);
      msTest.enableInterrupt(INT_TX | INT_RX);
      byte controllerFails = 0;
      controllerFails += testController(msTest, chan);
      boardFails += controllerFails;
      if(controllerFails > 0) {
        printSerialHeader(chan);
        Serial.println("Skipped test as controller non-functional.");
        continue;
      }
      boardFails += testSerialLoopback(msTest, chan);
      boardFails += testSerialLoopbackFifo(msTest, chan);
    }
    
    // channel doesn't matter for GPIOs, any channel selected goes to the same GPIO pins
    msTest = MultiSerial(boardAddr, 0);
    msTest.begin(9600);
    boardFails += testGpio(msTest);
    
    // print a summary so it's easy to tell if a board passes or not
    if(boardFails > 0) {
      Serial.println("\nBOARD FAIL: One or more tests failed, see above for details.");
    } else {
      Serial.println("\nBOARD PASS!");
    }
 /*   Serial.print("During the test, ");
    Serial.print(int_counter, DEC);
    Serial.print(" interrupts were triggered.\n\n");*/
  }
}

void loop() {
  delay(1000);
}
