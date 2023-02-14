/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2023 Sanworks LLC, Rochester, New York, USA

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
#include "ArCOM.h" // ArCOM is a serial interface wrapper developed by Sanworks, to streamline transmission of datatypes and arrays over serial

#define N_READS_PER_MEASUREMENT 20 // Number of fast digital input reads taken per channel per cycle. All must read high for the port to report 'in'.
                                   // Averaging 20 reads fixes interference with PWM dimming, and takes only ~10 additional microseconds per refresh cycle

ArCOM myUSB(Serial); // Creates an ArCOM object called myUSB, wrapping SerialUSB
ArCOM Serial1COM(Serial1); // Creates an ArCOM object called myUART, wrapping Serial1

uint32_t FirmwareVersion = 2;
char moduleName[] = "PA"; // Name of module for manual override UI and state machine assembler
char* eventNames[] = {"Port1In", "Port1Out", "Port2In", "Port2Out", "Port3In", "Port3Out", "Port4In", "Port4Out"};
byte nEventNames = (sizeof(eventNames)/sizeof(char *));
byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer

byte opCode = 0; 
byte opSource = 0;
boolean newOp = false;
const byte valveChannels[4] = {20, 21, 23, 22}; 
const byte ledChannels[4] = {3, 5, 9, 10};
const byte inputChannels[4] = {4, 6, 8, 11};

byte lastInputState[4] = {0};
byte inputState[4] = {0};
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
  for (int i = 0; i < 4; i++) {
    pinMode(inputChannels[i], INPUT_PULLDOWN);
    pinMode(valveChannels[i], OUTPUT);
    digitalWrite(valveChannels[i], 0);
    pinMode(ledChannels[i], OUTPUT);
    analogWriteFrequency(ledChannels[i], 50000); // Set LED PWM frequency (for dimming) to 50kHz
    analogWrite(ledChannels[i], 0);
    pinMode(13, OUTPUT); 
    digitalWrite(13, HIGH); // Turn on the board LED (as a power indicator)
  }
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
          myUSB.writeUint32(FirmwareVersion); // Send firmware version
          sessionStartTimeMicros = (uint64_t)microsTime;
        }
      break;
      case 'V': // Set state of a valve
        portID = readByteFromSource(opSource);
        newValue = readByteFromSource(opSource);
        digitalWrite(valveChannels[portID], newValue);
      break;
      case 'B': // Set Valve Array Bits (1 = open, 0 = closed)
        newValue = readByteFromSource(opSource);
        for (int i = 0; i < 4; i++) {
          if (bitRead(newValue, i)) {
            digitalWrite(valveChannels[i], HIGH);
          } else {
            digitalWrite(valveChannels[i], LOW);
          }
        }
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
        for (int i = 0; i < 4; i++) {
          newValue = readByteFromSource(opSource);
          analogWrite(ledChannels[i], newValue);
        }
        if (opSource == 0 && !usbStreaming) {
          myUSB.writeByte(1); // Confirm
        }
      break;
      case 'L': // Set LED Array Bits (1 = max brightness, 0 = off)
        newValue = readByteFromSource(opSource);
        for (int i = 0; i < 4; i++) {
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
  for (int i = 0; i < 4; i++) {
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
    myUSB.writeByteArray(usbEventBuffer.uint8, 12);
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

void returnModuleInfo() {
  Serial1COM.writeByte(65); // Acknowledge
  Serial1COM.writeUint32(FirmwareVersion); // 4-byte firmware version
  Serial1COM.writeByte(sizeof(moduleName)-1);
  Serial1COM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
  Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
  Serial1COM.writeByte('#'); // Op code for: Number of behavior events this module can generate
  Serial1COM.writeByte(8); // 4 Ports, 2 states each
  Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
  Serial1COM.writeByte('E'); // Op code for: Behavior event names
  Serial1COM.writeByte(nEventNames);
  for (int i = 0; i < nEventNames; i++) { // Once for each event name
    Serial1COM.writeByte(strlen(eventNames[i])); // Send event name length
    for (int j = 0; j < strlen(eventNames[i]); j++) { // Once for each character in this event name
      Serial1COM.writeByte(*(eventNames[i]+j)); // Send the character
    }
  }
  Serial1COM.writeByte(0); // 1 if more info follows, 0 if not
}
