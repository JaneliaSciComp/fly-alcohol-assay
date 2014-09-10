#include "SavedVariables.h"

SavedVariables::SavedVariables()
{
  initialize();
};

void SavedVariables::initialize() {
  // Check to see if EEPROM values have been initialized for the very first time, if not, initialize
  if (EEPROM.read(eepromAddressInitialized) != initializedValue) {
    EEPROM.write(eepromAddressInitialized,initializedValue);
    EEPROM.write(eepromAddressSerialNumber,constants::serialNumberMin);
    EEPROM.write(eepromAddressGateOpenServoAngle,constants::gateMinServoAngle);
    EEPROM.write(eepromAddressGateCloseServoAngle,constants::gateMaxServoAngle);
    EEPROM.write(eepromAddressLedOnDutyCycle,constants::ledMaxDutyCycle);
  }

  this->_serialNumber = EEPROM.read(eepromAddressSerialNumber);
  this->_gateOpenServoAngle = EEPROM.read(eepromAddressGateOpenServoAngle);
  this->_gateCloseServoAngle = EEPROM.read(eepromAddressGateCloseServoAngle);
  this->_ledOnDutyCycle = EEPROM.read(eepromAddressLedOnDutyCycle);
}

int SavedVariables::getSerialNumber() {
  return _serialNumber;
}

bool SavedVariables::setSerialNumber(int serial_number) {
  this->_serialNumber = serial_number;
  EEPROM.write(eepromAddressSerialNumber,serial_number);
  return true;
}

int SavedVariables::getGateOpenServoAngle() {
  return _gateOpenServoAngle;
}

bool SavedVariables::setGateOpenServoAngle(int angle) {
  this->_gateOpenServoAngle = angle;
  EEPROM.write(eepromAddressGateOpenServoAngle,angle);
  return true;
}

int SavedVariables::getGateCloseServoAngle() {
  return _gateCloseServoAngle;
}

bool SavedVariables::setGateCloseServoAngle(int angle) {
  this->_gateCloseServoAngle = angle;
  EEPROM.write(eepromAddressGateCloseServoAngle,angle);
  return true;
}

int SavedVariables::getLedOnDutyCycle() {
  return _ledOnDutyCycle;
}

bool SavedVariables::setLedOnDutyCycle(int duty_cycle) {
  this->_ledOnDutyCycle = duty_cycle;
  EEPROM.write(eepromAddressLedOnDutyCycle,duty_cycle);
  return true;
}

SavedVariables savedVariables;
