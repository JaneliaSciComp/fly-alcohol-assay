#include "Tunnel.h"
#include "Streaming.h"

Tunnel::Tunnel()
{
};

Tunnel::Tunnel(int tunnel):
  _tunnel(tunnel)
{
  initialize();
}

void Tunnel::initialize() {
  for (int gate=0; gate < constants::numGatesPerTunnel; gate++) {
    _gateServoArray[gate].attach(constants::gateServoPinArray[_tunnel][gate]);
    // immediately close gate to see if it helps startup jumpiness
    closeGate(gate);
    delay(constants::gateDelay);
  }
}

bool Tunnel::openGate(int gate) {
  if (!checkGateArg(gate)) {return false;};
  _gateServoArray[gate].write(savedVariables.getGateOpenServoAngle());
  return true;
}

bool Tunnel::closeGate(int gate) {
  if (!checkGateArg(gate)) {return false;};
  _gateServoArray[gate].write(savedVariables.getGateCloseServoAngle());
  return true;
}

bool Tunnel::openAllGates() {
  for (int gate=0; gate < constants::numGatesPerTunnel; gate++) {
    openGate(gate);
    delay(constants::gateDelay);
  }
  return true;
}

bool Tunnel::closeAllGates() {
  for (int gate=0; gate < constants::numGatesPerTunnel; gate++) {
    closeGate(gate);
    delay(constants::gateDelay);
  }
  return true;
}

bool Tunnel::setGateServoAngle(int gate, int angle) {
  if (!checkGateArg(gate) || !checkServoAngleArg(angle)) {return false;};
  _gateServoArray[gate].write(angle);
  return true;
}

bool Tunnel::setAllGatesServoAngle(int angle) {
  if (!checkServoAngleArg(angle)) {return false;};
  for (int gate=0; gate < constants::numGatesPerTunnel; gate++) {
    setGateServoAngle(gate,angle);
  }
  return true;
}

bool Tunnel::turnOnLed(int led) {
  if (!checkLedArg(led)) {return false;};
  Tlc.set(constants::ledChannelArray[_tunnel][led],convertDutyCycleToLedValue(savedVariables.getLedOnDutyCycle()));
  return true;
}

bool Tunnel::turnOffLed(int led) {
  if (!checkLedArg(led)) {return false;};
  Tlc.set(constants::ledChannelArray[_tunnel][led],constants::ledMinValue);
  return true;
}

bool Tunnel::turnOnAllLeds() {
  for (int led=0; led < constants::numLedsPerTunnel; led++) {
    turnOnLed(led);
  }
  return true;
}

bool Tunnel::turnOffAllLeds() {
  for (int led=0; led < constants::numLedsPerTunnel; led++) {
    turnOffLed(led);
  }
  return true;
}

bool Tunnel::setLedDutyCycle(int led, int duty_cycle) {
  if (!checkLedArg(led) || !checkLedDutyCycleArg(duty_cycle)) {return false;};
  int value = convertDutyCycleToLedValue(duty_cycle);
  Tlc.set(constants::ledChannelArray[_tunnel][led],value);
  return true;
}

bool Tunnel::setAllLedsDutyCycle(int duty_cycle) {
  if (!checkLedDutyCycleArg(duty_cycle)) {return false;};
  for (int led=0; led < constants::numLedsPerTunnel; led++) {
    setLedDutyCycle(led,duty_cycle);
  }
  return true;
}

bool Tunnel::checkGateArg(int gate) {
  bool flag = true;
  if ((gate<0) || (constants::numGatesPerTunnel<=gate)) {
    flag = false;
  }
  return flag;
}

bool Tunnel::checkLedArg(int led) {
  bool flag = true;
  if ((led<0) || (constants::numLedsPerTunnel<=led)) {
    flag = false;
  }
  return flag;
}

bool Tunnel::checkServoAngleArg(int angle) {
  bool flag = true;
  if ((angle<constants::gateMinServoAngle) || (constants::gateMaxServoAngle<angle)) {
    flag = false;
  }
  return flag;
}

bool Tunnel::checkLedDutyCycleArg(int duty_cycle) {
  bool flag = true;
  if ((duty_cycle<constants::ledMinDutyCycle) || (constants::ledMaxDutyCycle<duty_cycle)) {
    flag = false;
  }
  return flag;
}
