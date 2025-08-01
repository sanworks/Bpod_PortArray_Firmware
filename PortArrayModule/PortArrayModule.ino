/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2025 Sanworks LLC, Rochester, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <SPI.h>
#include "ArCOM.h" // ArCOM is a serial interface wrapper developed by Sanworks, to streamline transmission of datatypes and arrays over serial

#define FIRMWARE_VERSION 3

// SETUP MACROS TO COMPILE FOR TARGET DEVICE:
#define HARDWARE_VERSION 0 // Use: 1 = Port Array v1.X, 2 = Port Array v2.X
//-------------------------------------------

// Validate macros
#if (HARDWARE_VERSION < 1) || (HARDWARE_VERSION > 2)
#error Error! HARDWARE_VERSION must be either 1 or 2
#endif

ArCOM myUSB(Serial); // Creates an ArCOM object called myUSB, wrapping SerialUSB
ArCOM Serial1COM(Serial1); // Creates an ArCOM object called myUART, wrapping Serial1

char moduleName[] = "PA"; // Name of module for manual override UI and state machine assembler
#if (HARDWARE_VERSION == 1)
char* eventNames[] = {"Port1In", "Port1Out", "Port2In", "Port2Out", "Port3In", "Port3Out", "Port4In", "Port4Out"};
#else
char* eventNames[] = {"Port1In", "Port1Out", "Port2In", "Port2Out", "Port3In", "Port3Out", "Port4In", "Port4Out","Port5In", "Port5Out", "Port6In", "Port6Out", "Port7In", "Port7Out", "Port8In", "Port8Out"};
#endif
byte circuitRevision = 0; // Minor revision of circuit board
byte nEventNames = (sizeof(eventNames)/sizeof(char *));
byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer

byte opCode = 0; 
byte opSource = 0;
boolean newOp = false;
byte valveState = 0; 
const byte circuitRevisionArray[2] = {20,21};
const byte valveChannels[4] = {20, 21, 23, 22}; // Used for hardware v1 only

#if (HARDWARE_VERSION == 1)
  #define N_CHANNELS 4
  #define N_READS_PER_MEASUREMENT 250 // Number of fast digital input reads taken per channel per cycle. All must read high for the port to report 'in'.
                                    // Averaging 20 reads fixes interference with PWM dimming, and takes only ~10 additional microseconds per refresh cycle
  const byte ledChannels[4] = {3, 5, 9, 10};
  const byte inputChannels[4] = {4, 6, 8, 11};
#else
  #define N_CHANNELS 8
  #define N_READS_PER_MEASUREMENT 250
  const byte ledChannels[8] = {9, 10, 14, 15, 18, 19, 22, 23};
  const byte inputChannels[8] = {8, 7, 6, 5, 4, 3, 2, 17};
  const byte csPin = 16;
  SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE2);
#endif

byte lastInputState[N_CHANNELS] = {0};
byte inputState[N_CHANNELS] = {0};
byte portID = 0;
byte newValue = 0;
byte thisEvent = 0;
boolean usbStreaming = false;
boolean newEvent = false;

// Timing
uint64_t nMicrosRollovers = 0;
uint64_t sessionStartTimeMicros = 0;
uint64_t currentTime = 0;
uint32_t lastMicrosTime = 0;
uint32_t microsTime = 0;

union {
    byte uint8[16];
    uint64_t uint64[2];
} usbEventBuffer;

void setup() {
  Serial1.begin(1312500); 
  Serial1.addMemoryForRead(StateMachineSerialBuf, 192);
  for (int i = 0; i < N_CHANNELS; i++) {
    pinMode(inputChannels[i], INPUT_PULLDOWN);
    #if (HARDWARE_VERSION == 1)
      pinMode(valveChannels[i], OUTPUT);
      digitalWrite(valveChannels[i], 0);
    #endif
    pinMode(ledChannels[i], OUTPUT);
    analogWriteFrequency(ledChannels[i], 50000); // Set LED PWM frequency (for dimming) to 50kHz
    analogWrite(ledChannels[i], 0);
  }
  #if (HARDWARE_VERSION == 1)
    pinMode(13, OUTPUT); 
    digitalWrite(13, HIGH); // Turn on the board LED (as a power indicator)
  #else
    // Read hardware revision from circuit board (an array of grounded pins indicates revision in binary, grounded = 1, floating = 0)
    circuitRevision = 0;
    for (int i = 0; i < 2; i++) {
      pinMode(circuitRevisionArray[i], INPUT_PULLUP);
      circuitRevision += pow(2, i)*digitalRead(circuitRevisionArray[i]);
      pinMode(circuitRevisionArray[i], INPUT);
    }
    circuitRevision = 3-circuitRevision;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    SPI.begin();
    SPI.beginTransaction(spiSettings);
  #endif
}

void loop() {
  if (usbStreaming) {
    // Calculate 64-bit rollover-compensated time in microseconds
    microsTime = micros();
    currentTime = ((uint64_t)microsTime + (nMicrosRollovers*4294967295)) - sessionStartTimeMicros;
    if (microsTime < lastMicrosTime) {
      nMicrosRollovers++;
    }
    lastMicrosTime = microsTime;
    usbEventBuffer.uint64[0] = currentTime;
  }
  
  // Handle incoming byte messages
  if (myUSB.available()>0) {
    opCode = myUSB.readByte();
    opSource = 0; newOp = true;
  } else if (Serial1COM.available()) {
    opCode = Serial1COM.readByte();
    opSource = 1; newOp = true;
  }
  if (newOp) {
    newOp = false;
    switch (opCode) {
      case 255:
        if (opSource == 1) {
          returnModuleInfo();
        } else if (opSource == 0) {
          myUSB.writeByte(254); // Confirm
          myUSB.writeUint32(FIRMWARE_VERSION); // Send firmware version
          sessionStartTimeMicros = (uint64_t)microsTime;
        }
      break;
      case 'H': // Return hardware version
        myUSB.writeByte(HARDWARE_VERSION);
      break;
      case 'V': // Set state of a valve
        portID = readByteFromSource(opSource);
        newValue = readByteFromSource(opSource);
        setValve(portID, newValue);
      break;
      case 'B': // Set Valve Array Bits (1 = open, 0 = closed)
        newValue = readByteFromSource(opSource);
        setAllValves(newValue);
        if (opSource == 0 && !usbStreaming) {
          myUSB.writeByte(1); // Confirm
        }
      break;
      case 'P': // Set 1 LED PWM
        portID = readByteFromSource(opSource);
        newValue = readByteFromSource(opSource);
        analogWrite(ledChannels[portID], newValue);
        if (opSource == 0 && !usbStreaming) {
          myUSB.writeByte(1); // Confirm
        }
      break;
      case 'W': // Set All LED PWM
        for (int i = 0; i < N_CHANNELS; i++) {
          newValue = readByteFromSource(opSource);
          analogWrite(ledChannels[i], newValue);
        }
        if (opSource == 0 && !usbStreaming) {
          myUSB.writeByte(1); // Confirm
        }
      break;
      case 'L': // Set LED Array Bits (1 = max brightness, 0 = off)
        newValue = readByteFromSource(opSource);
        for (int i = 0; i < N_CHANNELS; i++) {
          if (bitRead(newValue, i)) {
            analogWrite(ledChannels[i], 256);
          } else {
            analogWrite(ledChannels[i], 0);
          }
        }
      break;
      case 'S': // Return current state of all ports to USB
        if (opSource == 0) {
          myUSB.writeByteArray(inputState, 4);
        }
      break;
      case 'U': // Start/Stop USB event stream
        if (opSource == 0) {
          usbStreaming = myUSB.readByte();
        }
      break;
      case 'R': // Reset clock
        sessionStartTimeMicros = (uint64_t)microsTime;
      break;
    }
  }
  thisEvent = 1;
  newEvent = false;
  usbEventBuffer.uint64[1] = 0; // Clear event portion of buffer
  for (int i = 0; i < N_CHANNELS; i++) {
    byte readCount = 0;
    for (int j = 0; j < N_READS_PER_MEASUREMENT; j++) {
      readCount += digitalReadFast(inputChannels[i]);
    }
    inputState[i] = LOW;
    if (readCount > N_READS_PER_MEASUREMENT/2) {
      inputState[i] = HIGH;
    }
    if (inputState[i] == HIGH) {
      if (lastInputState[i] == LOW) {
        Serial1COM.writeByte(thisEvent);
        usbEventBuffer.uint8[i+8] = thisEvent;
        newEvent = true;
      }
    }
    thisEvent++;
    if (inputState[i] == LOW) {
      if (lastInputState[i] == HIGH) {
        Serial1COM.writeByte(thisEvent);
        usbEventBuffer.uint8[i+8] = thisEvent;
        newEvent = true;
      }
    }
    thisEvent++; 
    lastInputState[i] = inputState[i];
  }
  if (newEvent && usbStreaming) {
    myUSB.writeByteArray(usbEventBuffer.uint8, 16);
  }
}

byte readByteFromSource(byte opSource) {
  switch (opSource) {
    case 0:
      return myUSB.readByte();
    break;
    case 1:
      return Serial1COM.readByte();
    break;
  }
}

void setValve(uint8_t portID, uint8_t newValue) {
  bitWrite(valveState, portID, newValue);
  #if (HARDWARE_VERSION == 1)
    digitalWrite(valveChannels[portID], newValue);
  #else
    digitalWriteFast(csPin, LOW);
    SPI.transfer(reverseBits(valveState)); // Reverse bits as hardware v2 IC channels are mapped backwards on the device
    digitalWriteFast(csPin, HIGH);
  #endif
}

void setAllValves(uint8_t newValue) {
  valveState = newValue;
  #if (HARDWARE_VERSION == 1)
    for (int i = 0; i < N_CHANNELS; i++) {
      if (bitRead(newValue, i)) {
          digitalWrite(valveChannels[i], HIGH);
      } else {
          digitalWrite(valveChannels[i], LOW);
      }
    }
  #else
    digitalWriteFast(csPin, LOW);
    SPI.transfer(reverseBits(valveState));
    digitalWriteFast(csPin, HIGH);
  #endif
}

uint8_t reverseBits(uint8_t b) { // Reverse bits as hardware v2 IC channels are mapped backwards on the device
  // The __RBIT ARM intrinsic efficiently reverses a 32-bit word. We cast our byte to a 32-bit word before the operation.
  uint32_t reversed_word = 0;
  __asm__("rbit %0, %1" : "=r" (reversed_word) : "r" ((uint32_t)b));
  // The 8 reversed bits are now in the MSB position of the 32-bit word.
  // We need to shift them right by 24 bits to get them back into a byte.
  return (uint8_t)(reversed_word >> 24);
}

void returnModuleInfo() {
  boolean fsmSupportsHwInfo = false;
  delayMicroseconds(100);
  if (Serial1COM.available() == 1) { // FSM firmware v23 or newer sends a second info request byte to indicate that it supports additional ops
    if (Serial1COM.readByte() == 255) {fsmSupportsHwInfo = true;}
  }
  Serial1COM.writeByte(65); // Acknowledge
  Serial1COM.writeUint32(FIRMWARE_VERSION); // 4-byte firmware version
  Serial1COM.writeByte(sizeof(moduleName)-1);
  Serial1COM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
  Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
  Serial1COM.writeByte('#'); // Op code for: Number of behavior events this module can generate
  Serial1COM.writeByte(N_CHANNELS*2); // V1: 4 Ports, 2 states each V2: 8 Ports, 2 states each
  Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
  Serial1COM.writeByte('E'); // Op code for: Behavior event names
  Serial1COM.writeByte(nEventNames);
  for (int i = 0; i < nEventNames; i++) { // Once for each event name
    Serial1COM.writeByte(strlen(eventNames[i])); // Send event name length
    for (int j = 0; j < strlen(eventNames[i]); j++) { // Once for each character in this event name
      Serial1COM.writeByte(*(eventNames[i]+j)); // Send the character
    }
  }
  if (fsmSupportsHwInfo) {
    Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
    Serial1COM.writeByte('V'); // Op code for: Hardware major version
    Serial1COM.writeByte(HARDWARE_VERSION); 
    Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
    Serial1COM.writeByte('v'); // Op code for: Hardware minor version
    Serial1COM.writeByte(circuitRevision); 
  }
  Serial1COM.writeByte(0); // 1 if more info follows, 0 if not
}
