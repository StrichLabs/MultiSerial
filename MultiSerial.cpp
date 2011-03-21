/*
  MultiSerial.cpp - Arduino library for the StrichLabs MultiSerial Shield
  Copyright (c) 2010 Sarah Nordstrom.  Licensed under Creative Commons BY-NC-SA.
*/

// include this library's description file
#include "MultiSerial.h"

// include description files for Wire, as the shield uses I2C
#include "Wire.h"

//#define MS_HZ 1843200L
#define   MS_HZ 11059200L
//#define MS_HZ   3686400L
  
// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

MultiSerial::MultiSerial(byte shieldAddress, byte myChannel) {
  addr = shieldAddress;
  chan = myChannel;
  peek_buf_valid[0] = 0;
  peek_buf_valid[1] = 0;
}

MultiSerial::MultiSerial(void) {
  // they can call the 'real' constructor above later
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries
void MultiSerial::begin(unsigned long baud) {
  // join i2c bus (no address as we want to be the master)
  Wire.begin();
  
  // reset the UART so we're starting off in a known state
  msWriteRegister(IOControl, 0x8);
  delay(25);
  
  // switch to the special registers
  msWriteRegister(LCR, 128);

  // set the baud rate
  unsigned short baudDivisor;
  baudDivisor = (MS_HZ + baud * 8L) / (baud * 16L);
  msWriteRegister(DLL, baudDivisor & 0xFF);
  msWriteRegister(DLH, baudDivisor >> 8);
  
  /*
  Serial.print("Sizeof: ");
  Serial.println(sizeof(baud), DEC);
  Serial.print("DLL: ");
  Serial.println(baudDivisor & 0xFF, DEC);
  Serial.print("DLH: ");
  Serial.println(baudDivisor >> 8, DEC);
  */
  
  // switch back to the normal registers
  msWriteRegister(LCR, 0);
  
  // set the word size to 8 bits
  setWordSize(8);
  
  // enable the TX/RX fifos
  msWriteRegister(FCR, 7);
}

void MultiSerial::write(uint8_t val) {
  msWriteRegister(THR, val);
}

int MultiSerial::read(void) {
  if(peek_buf_valid[chan]) {
    peek_buf_valid[chan] = 0;
    return peek_buf[chan];
  } else {
    return msReadRegister(RHR);
  }
}

int MultiSerial::available(void) {
  // FIXME: umm, what?  this only works if we read the register twice, for some reason...
//  msReadRegister(RXLVL);
  if(peek_buf_valid[chan]) return 1;
  return msReadRegister(RXLVL);
}

void MultiSerial::store(byte val) {
  msWriteRegister(SPR, val);
}

byte MultiSerial::retrieve(void) {
  return msReadRegister(SPR);
}

void MultiSerial::flush() {
  // needed for Stream support, but we don't need to do anything
  // FIXME: what? why not? it seems like we should flush the buffer..
}

int MultiSerial::peek() {
  if(peek_buf_valid[chan]) return peek_buf[chan];
  if(msReadRegister(RXLVL) < 1) return -1;
  peek_buf[chan] = msReadRegister(RHR);
  peek_buf_valid[chan] = 1;
  return peek_buf[chan];
}

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

byte MultiSerial::digitalRead(byte pin) {
  byte regValues;
  
  regValues = msReadRegister(IOState);
  if((regValues & (1 << pin)) > 0) {
    return HIGH;
  } else {
    return LOW;
  }
}

void MultiSerial::setWordSize(byte wordSize) {
  byte curValue;
  
  wordSize -= 5;
  curValue = msReadRegister(LCR);
  msWriteRegister(LCR, curValue | wordSize);
}

void MultiSerial::enableInterrupt(void) {
  msWriteRegister(IER, 1);
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