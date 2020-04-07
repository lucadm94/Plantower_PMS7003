/*********************************************************************
An Arduino library for interfacing with the Plantower PMS7003 laser
particle counter. This should also work the the PMS5003 sensor,
as they share the same protocol.

For more information about the PMS7003, see README.md.

Written by Jason Striegel.
BSD license. See license.txt for details.
**********************************************************************/

#include "Plantower_PMS7003.h"
#include "../../../tools/log/can5_log.h"

Plantower_PMS7003::Plantower_PMS7003() {
  dataReady = false;
  initialized = false;
  debug = false;
}

void Plantower_PMS7003::init(Stream *s) {
  dataReady = false;
  serial = s;
  nextByte = 0;
  lastByte = 0;
  bufferIndex = 0;
  initialized = true;
}

void Plantower_PMS7003::updateFrame() {
  if (!initialized) {
    DDEBUGLN("Error: must call Plantower_PMS7003::init()");
    return;
  }
  dataReady = false;
  if (serial->available()) {
    nextByte = serial->read();
    
    if (nextByte == 0x4d && lastByte == 0x42 ) {
      sensorData.bytes[0] = 0x42;
      bufferIndex=1;
    }
    sensorData.bytes[bufferIndex] = nextByte;    
    bufferIndex++;
    lastByte = nextByte;
  }

  if (bufferIndex == PMS7003_DATA_SIZE) {
    if (sensorData.bytes[0] == 0x42 && sensorData.bytes[1] == 0x4d) {
      
      convertSensorData();
      
      if(isValidChecksum()) {
        dataReady = true;
      } else {
        if (debug) {
          DDEBUGLN("Invalid data checksum");
        }
      }
    } else {
      if (debug) {
        DDEBUGLN("Malformed first byte");
      }
    }
    bufferIndex=0;
  }   

}

bool Plantower_PMS7003::hasNewData() {
  return dataReady;
}


uint16_t Plantower_PMS7003::getPM_1_0() {
  return sensorData.values.pm_1_0;
}
uint16_t Plantower_PMS7003::getPM_2_5() {
  return sensorData.values.pm_2_5;
}
uint16_t Plantower_PMS7003::getPM_10_0() {
  return sensorData.values.pm_10_0;
}
uint16_t Plantower_PMS7003::getPM_1_0_atmos() {
  return sensorData.values.pm_1_0_atmos;
}
uint16_t Plantower_PMS7003::getPM_2_5_atmos() {
  return sensorData.values.pm_2_5_atmos;
}
uint16_t Plantower_PMS7003::getPM_10_0_atmos() {
  return sensorData.values.pm_10_0_atmos;
}

uint16_t Plantower_PMS7003::getRawGreaterThan_0_3() {
  return sensorData.values.raw_gt_0_3;
}
uint16_t Plantower_PMS7003::getRawGreaterThan_0_5() {
  return sensorData.values.raw_gt_0_5;
}
uint16_t Plantower_PMS7003::getRawGreaterThan_1_0() {
  return sensorData.values.raw_gt_1_0;
}
uint16_t Plantower_PMS7003::getRawGreaterThan_2_5() {
  return sensorData.values.raw_gt_2_5;
}
uint16_t Plantower_PMS7003::getRawGreaterThan_5_0() {
  return sensorData.values.raw_gt_5_0;
}
uint16_t Plantower_PMS7003::getRawGreaterThan_10_0() {
  return sensorData.values.raw_gt_10_0;
}

uint8_t Plantower_PMS7003::getHWVersion() {
  return sensorData.values.version_number;
}
uint8_t Plantower_PMS7003::getErrorCode() {
  return sensorData.values.error_code;
}

// fix sensor data endianness
void Plantower_PMS7003::convertSensorData() {
  int loc=0;

  // first word (bytes 0 and 1) are 8 bit
  loc += 1;
  
  // next 13 words are 16-bit ints
  for (int i=0; i<13; i++) {
    sensorData.words[loc] = uint16FromBufferData(sensorData.bytes, loc*2);
    loc++;
  }
  
  // next word is 2 8-bit bytes
  loc += 1;
  
  // final checksum word is 16-bit
  sensorData.words[loc] = uint16FromBufferData(sensorData.bytes, loc*2);
}

bool Plantower_PMS7003::isValidChecksum() {
  uint16_t sum = 0;
  
  for(int i=0; i<(PMS7003_DATA_SIZE-2); i++){
    sum += sensorData.bytes[i];
  }
  if(sum == sensorData.values.checksum)
  {
    return 1;
  }
  return 0;
}

uint16_t Plantower_PMS7003::uint16FromBufferData(unsigned char *buff, int loc) {
  if (loc > PMS7003_DATA_SIZE-2 || loc < 2) {
    return -1;
  }
  return ((buff[loc]<<8) + buff[loc+1]);
}



// Standby mode. For low power consumption and prolong the life of the sensor.
void Plantower_PMS7003::sleep()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };
  serial->write(command, sizeof(command));
}

// Operating mode. Stable data should be got at least 30 seconds after the sensor wakeup from the sleep mode because of the fan's performance.
void Plantower_PMS7003::wakeUp()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };
  serial->write(command, sizeof(command));
}

// Active mode. Default mode after power up. In this mode sensor would send serial data to the host automatically.
void Plantower_PMS7003::activeMode()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
  serial->write(command, sizeof(command));
  _mode = MODE_ACTIVE;
}

// Passive mode. In this mode sensor would send serial data to the host only for request.
void Plantower_PMS7003::passiveMode()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 };
  serial->write(command, sizeof(command));
  _mode = MODE_PASSIVE;
}

// Request read in Passive Mode.
void Plantower_PMS7003::requestRead()
{
  if (_mode == MODE_PASSIVE)
  {
    uint8_t command[] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 };
    serial->write(command, sizeof(command));
  }
}