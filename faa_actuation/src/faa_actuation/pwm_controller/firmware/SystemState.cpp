#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "string.h"
#include "SystemState.h"

SystemState::SystemState() {
  setErrMsg("");
}

void SystemState::setErrMsg(char *msg) {
  strncpy(errMsg,msg,SYS_ERR_BUF_SZ);
}

void SystemState::initialize() {
  Tlc.init();

  // Assign servo pins and led channels to tunnels and initialize
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    _tunnelArray[tunnel] = Tunnel(tunnel);
  }
  // closeAllGates();
  turnOffAllLeds();
}

bool SystemState::openGate(int tunnel, int gate) {
  if (!checkTunnelArg(tunnel)) {return false;};
  return _tunnelArray[tunnel].openGate(gate);
}

bool SystemState::closeGate(int tunnel, int gate) {
  if (!checkTunnelArg(tunnel)) {return false;};
  return _tunnelArray[tunnel].closeGate(gate);
}

bool SystemState::openAllGatesByGate(int gate) {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].openGate(gate);
    delay(constants::gateDelay);
  }
  return flag;
}

bool SystemState::closeAllGatesByGate(int gate) {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].closeGate(gate);
    delay(constants::gateDelay);
  }
  return flag;
}

bool SystemState::openAllGatesByTunnel(int tunnel) {
  if (!checkTunnelArg(tunnel)) {return false;};
  return _tunnelArray[tunnel].openAllGates();
}

bool SystemState::closeAllGatesByTunnel(int tunnel) {
  if (!checkTunnelArg(tunnel)) {return false;};
  return _tunnelArray[tunnel].closeAllGates();
}

bool SystemState::openAllGates() {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].openAllGates();
    delay(constants::gateDelay);
  }
  return flag;
}

bool SystemState::closeAllGates() {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].closeAllGates();
    delay(constants::gateDelay);
  }
  return flag;
}

bool SystemState::setGateServoAngle(int tunnel, int gate, int angle) {
  if (!checkTunnelArg(tunnel)) {return false;};
  return _tunnelArray[tunnel].setGateServoAngle(gate,angle);
}

bool SystemState::setAllGatesServoAngleByGate(int gate, int angle) {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].setGateServoAngle(gate,angle);
  }
  return flag;
}

bool SystemState::setAllGatesServoAngleByTunnel(int tunnel, int angle) {
  if (!checkTunnelArg(tunnel)) {return false;};
  return _tunnelArray[tunnel].setAllGatesServoAngle(angle);
}

bool SystemState::setAllGatesServoAngle(int angle) {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].setAllGatesServoAngle(angle);
  }
  return flag;
}

bool SystemState::turnOnLed(int tunnel, int led) {
  if (!checkTunnelArg(tunnel)) {return false;};
  bool flag = _tunnelArray[tunnel].turnOnLed(led);
  Tlc.update();
  return flag;
}

bool SystemState::turnOffLed(int tunnel, int led) {
  if (!checkTunnelArg(tunnel)) {return false;};
  bool flag = _tunnelArray[tunnel].turnOffLed(led);
  Tlc.update();
  return flag;
}

bool SystemState::turnOnAllLedsByLed(int led) {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].turnOnLed(led);
  }
  Tlc.update();
  return flag;
}

bool SystemState::turnOffAllLedsByLed(int led) {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].turnOffLed(led);
  }
  Tlc.update();
  return flag;
}

bool SystemState::turnOnAllLedsByTunnel(int tunnel) {
  if (!checkTunnelArg(tunnel)) {return false;};
  bool flag = _tunnelArray[tunnel].turnOnAllLeds();
  Tlc.update();
  return flag;
}

bool SystemState::turnOffAllLedsByTunnel(int tunnel) {
  if (!checkTunnelArg(tunnel)) {return false;};
  bool flag = _tunnelArray[tunnel].turnOffAllLeds();
  Tlc.update();
  return flag;
}

bool SystemState::turnOnAllLeds() {
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].turnOnAllLeds();
  }
  Tlc.update();
  return flag;
}

bool SystemState::turnOffAllLeds() {
  bool flag = true;
  Tlc.setAll(constants::ledMinValue);
  Tlc.update();
  return flag;
}

bool SystemState::setLedDutyCycle(int tunnel, int led, int duty_cycle) {
  if (!checkTunnelArg(tunnel) || !checkDutyCycleArg(duty_cycle)) {return false;};
  bool flag = _tunnelArray[tunnel].setLedDutyCycle(led,duty_cycle);
  Tlc.update();
  return flag;
}

bool SystemState::setAllLedsDutyCycleByLed(int led, int duty_cycle) {
  if (!checkDutyCycleArg(duty_cycle)) {return false;};
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].setLedDutyCycle(led,duty_cycle);
  }
  Tlc.update();
  return flag;
}

bool SystemState::setAllLedsDutyCycleByTunnel(int tunnel, int duty_cycle) {
  if (!checkTunnelArg(tunnel) || !checkDutyCycleArg(duty_cycle)) {return false;};
  bool flag = _tunnelArray[tunnel].setAllLedsDutyCycle(duty_cycle);
  Tlc.update();
  return flag;
}

bool SystemState::setAllLedsDutyCycle(int duty_cycle) {
  if (!checkDutyCycleArg(duty_cycle)) {return false;};
  bool flag = true;
  for (int tunnel=0; tunnel<constants::numTunnels; tunnel++) {
    flag = _tunnelArray[tunnel].setAllLedsDutyCycle(duty_cycle);
  }
  Tlc.update();
  return flag;
}

bool SystemState::checkTunnelArg(int tunnel) {
  bool flag = true;
  if ((tunnel<0) || (constants::numTunnels<=tunnel)) {
    flag = false;
  }
  return flag;
}

bool SystemState::checkDutyCycleArg(int duty_cycle) {
  bool flag = true;
  if ((duty_cycle<constants::ledMinDutyCycle) || (constants::ledMaxDutyCycle<duty_cycle)) {
    flag = false;
  }
  return flag;
}

SystemState systemState;


