// SavedVariables.h
#ifndef SavedVariables_h
#define SavedVariables_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif
#include <EEPROM.h>
#include "constants.h"


class SavedVariables {

 public:

  SavedVariables();
  int getSerialNumber();
  bool setSerialNumber(int serial_number);
  int getGateOpenServoAngle();
  bool setGateOpenServoAngle(int angle);
  int getGateCloseServoAngle();
  bool setGateCloseServoAngle(int angle);
  int getLedOnDutyCycle();
  bool setLedOnDutyCycle(int duty_cycle);

 private:

  enum {eepromAddressInitialized = 0};
  enum {eepromAddressSerialNumber = 1};
  enum {eepromAddressGateOpenServoAngle = 2};
  enum {eepromAddressGateCloseServoAngle = 3};
  enum {eepromAddressLedOnDutyCycle = 4};

  enum {initializedValue = 123};

  int _serialNumber;
  int _gateOpenServoAngle;
  int _gateCloseServoAngle;
  int _ledOnDutyCycle;

  void initialize();

};

extern SavedVariables savedVariables;

#endif
