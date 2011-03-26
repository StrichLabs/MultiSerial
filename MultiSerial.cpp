/*
  MultiSerial.cpp - Arduino library for the StrichLabs MultiSerial Shield
  Copyright (c) 2010 Sarah Nordstrom.  Licensed under Creative Commons BY-NC-SA.
*/

// include this library's description file
#include "MultiSerial.h"

// include description files for Wire, as the shield uses I2C
#include "Wire.h"

// Revision 1 boards have a 1.8432MHz crystal on them
//#define MS_HZ 1843200L
// Revision 2 boards have a 3.6864MHz crystal on them
//#define MS_HZ   3686400L
// Revision 3 and above boards have a 11.0592MHz crystal on them
#define   MS_HZ 11059200L
  
// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

// Description: Creates a variable of type MultiSerial, to communciate with the MultiSerial Shield.
// Syntax: MultiSerial(address, port)
// Parameter: address - Address on the I2C bus of the shield to communicate with
// Parameter: port - Which port (0 or 1) on the shield to associate this instance with
// Returns: Instance of MultiSerial associated with a specific port on a specific shield. 
MultiSerial::MultiSerial(byte shieldAddress, byte myChannel) {
  addr = shieldAddress;
  chan = myChannel;
  
  // each channel gets a one byte 'peek buffer', to support peek() even though
  // the hardware itself has no ability to peek into the buffer
  peek_buf_valid[0] = 0;
  peek_buf_valid[1] = 0;
}

MultiSerial::MultiSerial(void) {
  // this 'fake constructor' allows creating an 'empty' MultiSerial type variable
  // later on, the 'real constructor' can be called to actually begin using it.
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

// Description: Sets the data rate in bits per second for serial data transmission.
// Syntax: MultiSerialInstance.begin(speed)
// Parameter: speed - communications speed in bits per second
// Returns: nothing
void MultiSerial::begin(unsigned long baud) {
  // join i2c bus (no address as we want to be the master)
  Wire.begin();
  
  // reset the UART so we're starting off in a known state
  msWriteRegister(IOControl, 0x8);
  delay(25);
  
  // switch to the special register bank
  msWriteRegister(LCR, 128);

  // set the baud rate
  unsigned short baudDivisor;
  baudDivisor = (MS_HZ + baud * 8L) / (baud * 16L);
  msWriteRegister(DLL, baudDivisor & 0xFF);
  msWriteRegister(DLH, baudDivisor >> 8);
    
  // switch back to the normal register bank
  msWriteRegister(LCR, 0);
  
  // set the word size to 8 bits
  setWordSize(8);
  
  // enable the TX/RX fifos
  msWriteRegister(FCR, 7);
}

// Description: Transmit a byte of data out the serial port.
// Syntax: MultiSerialInstance.write(data)
// Parameter: data - byte of data to write
// Returns: nothing
// FIXME: This function should return something useful
void MultiSerial::write(uint8_t val) {
  msWriteRegister(THR, val);
}

// Description: Read the next available byte of available serial data.
// Syntax: MultiSerialInstance.read()
// Parameter: none
// Returns: first available byte of serial data, or -1 if no data is available
int MultiSerial::read(void) {
  if(peek_buf_valid[chan]) {
    peek_buf_valid[chan] = 0;
    return peek_buf[chan];
  } else {
    return msReadRegister(RHR);
  }
}

// Description: Get the number of bytes available for reading from the serial port.
// Syntax: MultiSerialInstance.available()
// Parameter: none
// Returns: number of bytes available to read
int MultiSerial::available(void) {
  if(peek_buf_valid[chan]) return 1;
  return msReadRegister(RXLVL);
}

// Description: Store one byte of data associated with a specific shield and port.  This byte is stored on the shield itself, not in the Arduino's memory.  Each successive call to store() will overwrite any previous byte stored.
// Syntax: MultiSerialInstance.store(data)
// Parameter: one byte of data to associate with a specific port on a specific shield
// Returns: nothing
void MultiSerial::store(byte val) {
  msWriteRegister(SPR, val);
}

// Description: Retrieve the byte of data aassociated with a specific shield and port, that had been stored earlier.
// Syntax: MultiSerialInstance.retrieve()
// Parameter: none
// Returns: one byte of data that is associated with a specific port on a specific shield
byte MultiSerial::retrieve(void) {
  return msReadRegister(SPR);
}

// Description: Flush all waiting data out of the buffer.
// Syntax: MultiSerialInstance.flush()
// Parameter: none
// Returns: nothing
void MultiSerial::flush() {
  msWriteRegister(FCR, 6);    // flush receive FIFO
  peek_buf_valid[chan] = 0;   // invalidate peek buffer  
}

// Description: Read the next available byte of available serial data, but leave it in the buffer.  Every call to peek() will return the same byte of data, until read() is called to return/remove it from the buffer or flush() is called to purge it.
// Syntax: MultiSerialInstance.peek()
// Parameter: none
// Returns: first available byte of serial data, or -1 if no data is available
int MultiSerial::peek() {
  if(peek_buf_valid[chan]) return peek_buf[chan];
  if(msReadRegister(RXLVL) < 1) return -1;
  peek_buf[chan] = msReadRegister(RHR);
  peek_buf_valid[chan] = 1;
  return peek_buf[chan];
}

// Description: Configured the specified GPIO pin to be an input or an output.
// Syntax: MultiSerialInstance.pinMode(pin, mode);
// Parameter: pin - GPIO pin number to change
// Parameter: mode - either INPUT or OUTPUT
// Returns: nothing
void MultiSerial::pinMode(byte pin, byte direction) {
  byte regDirs;

  regDirs = msReadRegister(IODir);
  if(direction == OUTPUT) {
    regDirs = regDirs | (1 << pin);
  } else {
    msWriteRegister(IODir, 0);
    regDirs = regDirs & ~(1 << pin);
  }
  msWriteRegister(IODir, regDirs);
}

// Description: Set a specified GPIO pin to a HIGH or LOW state.  Unlike the Arduino digital pins, these do not have pullup resistors, so digitalWrite when the pin is set to an input has no effect.
// Syntax: MultiSerialInstance.digitalWrite(pin, value)
// Parameter: pin - GPIO pin number to change
// Parameter: value - HIGH or LOW
// Returns: nothing
void MultiSerial::digitalWrite(byte pin, byte value) {
  byte regValues;
  
  regValues = msReadRegister(IOState);
  if(value == HIGH) {
    regValues = regValues | (1 << pin);
  } else {
    regValues = regValues & ~(1 << pin);
  }
  msWriteRegister(IOState, regValues);
}

// Description: Read the current value of a specific GPIO pin.
// Syntax: MultiSerialInstance.digitalRead(pin)
// Parameter: pin - GPIO pin number to read the value of
// Returns: HIGH or LOW
byte MultiSerial::digitalRead(byte pin) {
  byte regValues;
  
  regValues = msReadRegister(IOState);
  if((regValues & (1 << pin)) > 0) {
    return HIGH;
  } else {
    return LOW;
  }
}

// Description: Set the data word size to use on a specific serial port.  Defaults to 8 bits when begin() is called, but can be changed if required.
// Syntax: MultiSerialInstance.setWordSize(size)
// Parameter: size - word size to use on the port, valid values are 5, 6, 7, or 8
// Returns: nothing
void MultiSerial::setWordSize(byte wordSize) {
  byte curValue;
  
  wordSize -= 5;
  curValue = msReadRegister(LCR);
  msWriteRegister(LCR, curValue | wordSize);
}

// Description: Enable specific interrupt types.
// Syntax: MultiSerialInstance.enableInterrupt(types)
// Parameter: types - bitmask of INT_RX and/or INT_TX for which interrupt(s) to enable
// Returns: nothing
void MultiSerial::enableInterrupt(byte interruptTypes) {
  byte curIntFlags = msReadRegister(IER) & (INT_RX | INT_TX);
  msWriteRegister(IER, (interruptTypes | curIntFlags) & (INT_RX | INT_TX);
}

// Description: Disable specific interrupt types.
// Syntax: MultiSerialInstance.disableInterrupt(types)
// Parameter: types - bitmask of INT_RX and/or INT_TX for which interrupt(s) to disable
// Returns: nothing
void MultiSerial::disableInterrupt(byte interruptTypes) {
  byte curIntFlags = msReadRegister(IER) & (INT_RX | INT_TX);
  msWriteRegister(IER, (interruptTypes ^ curIntFlags) & (INT_RX | INT_TX);
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

// send the subaddress required to access one of the controller chip's registers
void MultiSerial::msSendSubAddr(byte reg) {
  int subAddr = 0;
  
  subAddr |= (chan << 1);
  subAddr |= (reg << 3);
  Wire.send(subAddr);
}

// write a value to one of the controller chip's registers
void MultiSerial::msWriteRegister(byte reg, byte val) {
  // begin the i2c transmission to the 
  Wire.beginTransmission(addr);
  // send the register address we want to write to
  msSendSubAddr(reg);
  // send the actual data we want to write and commit the transaction
  Wire.send(val);
  Wire.endTransmission();
  // TODO: is this needed?
  delay(10);
}

// read a value from one of the controller chip's registers and return it
byte MultiSerial::msReadRegister(byte reg) {
  Wire.beginTransmission(addr);
  msSendSubAddr(reg);
  Wire.endTransmission();
  // TODO: is this needed?
  delay(10);
  Wire.requestFrom((byte)addr, (byte)1);
  // TODO: is this needed?
  delay(10);
  return Wire.receive();
}

// Documentation ///////////////////////////////////////////////////////////////

// Description: Outputs data to the serial port, converting numbers to their human-readable ASCII representations.
// Syntax: MultiSerialInstance.print(value)
// Syntax: MultiSerialInstance.print(value, base)
// Syntax: MultiSerialInstance.print(string)
// Parameter: value - Variable or literal to output
// Parameter: base - Base to display number in (OCT, DEC, BIN, HEX) or number of decimal places for floating point values, DEC assumed if not supplied
// Parameter: string - String to output
// Returns: nothing
// void MultiSerial::print(int value, byte base) {}