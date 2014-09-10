#include <util/atomic.h>
#include "MessageHandler.h"
#include "Streaming.h"
#include "SystemState.h"
#include "Array.h"

enum {
  cmdGetDevInfo,                    // Done
  cmdGetCmds,                       // Done
  cmdGetRspCodes,                   // Done
  cmdSetSerialNumber,               //

  cmdSetGateOpenServoAngle,         //
  cmdSetGateCloseServoAngle,        //
  cmdSetLedOnDutyCycle,             //

  cmdOpenGate,
  cmdCloseGate,
  cmdOpenAllGatesByGate,
  cmdCloseAllGatesByGate,
  cmdOpenAllGatesByTunnel,
  cmdCloseAllGatesByTunnel,
  cmdOpenAllGates,                  // Done
  cmdCloseAllGates,                 // Done
  cmdSetGateServoAngle,             // Done
  cmdSetAllGatesServoAngleByGate,   // Done
  cmdSetAllGatesServoAngleByTunnel, // Done
  cmdSetAllGatesServoAngle,         // Done

  cmdTurnOnAllLeds,                 // Done
  cmdTurnOffAllLeds,                // Done
  cmdTurnOnAllLedsByLed,            // Done
  cmdTurnOffAllLedsByLed,           // Done
  cmdTurnOnAllLedsByTunnel,         // Done
  cmdTurnOffAllLedsByTunnel,        // Done
  cmdTurnOnLed,                     // Done
  cmdTurnOffLed,                    // Done
  cmdSetLedDutyCycle,               // Done
  cmdSetAllLedsDutyCycleByLed,      // Done
  cmdSetAllLedsDutyCycleByTunnel,   // Done
  cmdSetAllLedsDutyCycle,           // Done

  // DEVELOPMENT
  cmdDebug,
};


const int rspSuccess = 1;
const int rspError = 0;

void MessageHandler::processMsg() {
  while (Serial.available() > 0) {
    process(Serial.read());
    if (messageReady()) {
      msgSwitchYard();
      reset();
    }
  }
  return;
}

void MessageHandler::msgSwitchYard() {
  int cmd = readInt(0);
  dprint.start();
  dprint.addIntItem("cmd_id", cmd);

  switch (cmd) {

  case cmdGetDevInfo:
    handleGetDevInfo();
    break;

  case cmdGetCmds:
    handleGetCmds();
    break;

  case cmdGetRspCodes:
    handleGetRspCodes();
    break;

  case cmdSetSerialNumber:
    handleSetSerialNumber();
    break;

  case cmdSetGateOpenServoAngle:
    handleSetGateOpenServoAngle();
    break;

  case cmdSetGateCloseServoAngle:
    handleSetGateCloseServoAngle();
    break;

  case cmdSetLedOnDutyCycle:
    handleSetLedOnDutyCycle();
    break;

  case cmdOpenGate:
    handleOpenGate();
    break;

  case cmdCloseGate:
    handleCloseGate();
    break;

  case cmdOpenAllGatesByGate:
    handleOpenAllGatesByGate();
    break;

  case cmdCloseAllGatesByGate:
    handleCloseAllGatesByGate();
    break;

  case cmdOpenAllGatesByTunnel:
    handleOpenAllGatesByTunnel();
    break;

  case cmdCloseAllGatesByTunnel:
    handleCloseAllGatesByTunnel();
    break;

  case cmdOpenAllGates:
    handleOpenAllGates();
    break;

  case cmdCloseAllGates:
    handleCloseAllGates();
    break;

  case cmdSetGateServoAngle:
    handleSetGateServoAngle();
    break;

  case cmdSetAllGatesServoAngleByGate:
    handleSetAllGatesServoAngleByGate();
    break;

  case cmdSetAllGatesServoAngleByTunnel:
    handleSetAllGatesServoAngleByTunnel();
    break;

  case cmdSetAllGatesServoAngle:
    handleSetAllGatesServoAngle();
    break;

  case cmdTurnOnAllLeds:
    handleTurnOnAllLeds();
    break;

  case cmdTurnOffAllLeds:
    handleTurnOffAllLeds();
    break;

  case cmdTurnOnAllLedsByLed:
    handleTurnOnAllLedsByLed();
    break;

  case cmdTurnOffAllLedsByLed:
    handleTurnOffAllLedsByLed();
    break;

  case cmdTurnOnAllLedsByTunnel:
    handleTurnOnAllLedsByTunnel();
    break;

  case cmdTurnOffAllLedsByTunnel:
    handleTurnOffAllLedsByTunnel();
    break;

  case cmdTurnOnLed:
    handleTurnOnLed();
    break;

  case cmdTurnOffLed:
    handleTurnOffLed();
    break;

  case cmdSetLedDutyCycle:
    handleSetLedDutyCycle();
    break;

  case cmdSetAllLedsDutyCycleByLed:
    handleSetAllLedsDutyCycleByLed();
    break;

  case cmdSetAllLedsDutyCycleByTunnel:
    handleSetAllLedsDutyCycleByTunnel();
    break;

  case cmdSetAllLedsDutyCycle:
    handleSetAllLedsDutyCycle();
    break;

  // DEVELOPMENT
  case cmdDebug:
    handleDebug();
    break;

  default:
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "unknown command");
    break;
  }
  dprint.stop();
}

void MessageHandler::handleGetCmds() {
  dprint.addIntItem("status", rspSuccess);
  dprint.addIntItem("getDevInfo", cmdGetDevInfo);
  dprint.addIntItem("getCmds", cmdGetCmds);
  dprint.addIntItem("getRspCodes", cmdGetRspCodes);
  dprint.addIntItem("setArduinoSerialNumber", cmdSetSerialNumber);

  dprint.addIntItem("setGateOpenServoAngle", cmdSetGateOpenServoAngle);
  dprint.addIntItem("setGateCloseServoAngle", cmdSetGateCloseServoAngle);
  dprint.addIntItem("setLedOnDutyCycle", cmdSetLedOnDutyCycle);

  dprint.addIntItem("openGate", cmdOpenGate);
  dprint.addIntItem("closeGate", cmdCloseGate);
  dprint.addIntItem("openAllGatesByGate", cmdOpenAllGatesByGate);
  dprint.addIntItem("closeAllGatesByGate", cmdCloseAllGatesByGate);
  dprint.addIntItem("openAllGatesByTunnel", cmdOpenAllGatesByTunnel);
  dprint.addIntItem("closeAllGatesByTunnel", cmdCloseAllGatesByTunnel);
  dprint.addIntItem("openAllGates", cmdOpenAllGates);
  dprint.addIntItem("closeAllGates", cmdCloseAllGates);
  dprint.addIntItem("setGateServoAngle", cmdSetGateServoAngle);
  dprint.addIntItem("setAllGatesServoAngleByGate", cmdSetAllGatesServoAngleByGate);
  dprint.addIntItem("setAllGatesServoAngleByTunnel", cmdSetAllGatesServoAngleByTunnel);
  dprint.addIntItem("setAllGatesServoAngle", cmdSetAllGatesServoAngle);

  dprint.addIntItem("turnOnAllLeds", cmdTurnOnAllLeds);
  dprint.addIntItem("turnOffAllLeds", cmdTurnOffAllLeds);
  dprint.addIntItem("turnOnAllLedsByLed", cmdTurnOnAllLedsByLed);
  dprint.addIntItem("turnOffAllLedsByLed", cmdTurnOffAllLedsByLed);
  dprint.addIntItem("turnOnAllLedsByTunnel", cmdTurnOnAllLedsByTunnel);
  dprint.addIntItem("turnOffAllLedsByTunnel", cmdTurnOffAllLedsByTunnel);
  dprint.addIntItem("turnOnLed", cmdTurnOnLed);
  dprint.addIntItem("turnOffLed", cmdTurnOffLed);
  dprint.addIntItem("setLedDutyCycle", cmdSetLedDutyCycle);
  dprint.addIntItem("setAllLedsDutyCycleByLed", cmdSetAllLedsDutyCycleByLed);
  dprint.addIntItem("setAllLedsDutyCycleByTunnel", cmdSetAllLedsDutyCycleByTunnel);
  dprint.addIntItem("setAllLedsDutyCycle", cmdSetAllLedsDutyCycle);
  // DEVELOPMENT
  dprint.addIntItem("cmdDebug", cmdDebug);
}

void MessageHandler::handleGetDevInfo() {
  dprint.addIntItem("status", rspSuccess);
  dprint.addIntItem("model_number",  constants::modelNumber);
  dprint.addIntItem("serial_number", savedVariables.getSerialNumber());
  dprint.addIntItem("firmware_number", constants::firmwareNumber);
  dprint.addIntItem("tunnel_count",  constants::numTunnels);
  dprint.addIntItem("gate_per_tunnel_count",  constants::numGatesPerTunnel);
  dprint.addIntItem("led_per_tunnel_count",  constants::numLedsPerTunnel);
  dprint.addIntItem("gate_open_servo_angle",  savedVariables.getGateOpenServoAngle());
  dprint.addIntItem("gate_close_servo_angle",  savedVariables.getGateCloseServoAngle());
  dprint.addIntItem("gate_min_servo_angle",  constants::gateMinServoAngle);
  dprint.addIntItem("gate_max_servo_angle",  constants::gateMaxServoAngle);
  dprint.addIntItem("led_on_duty_cycle",  savedVariables.getLedOnDutyCycle());
}

bool MessageHandler::checkNumberOfArgs(int num) {
  bool flag = true;
  if (numberOfItems() != num) {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "incorrect number of arguments");
    flag = false;
  }
  return flag;
}

bool MessageHandler::checkSerialNumberArg(int serial_number) {
  bool flag = true;
  if ((serial_number<constants::serialNumberMin) || (constants::serialNumberMax<serial_number)) {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "serial number argument out of range");
    flag = false;
  }
  return flag;
}

bool MessageHandler::checkTunnelArg(int tunnel) {
  bool flag = true;
  if ((tunnel<0) || (constants::numTunnels<=tunnel)) {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "tunnel argument out of range");
    flag = false;
  }
  return flag;
}

bool MessageHandler::checkGateArg(int gate) {
  bool flag = true;
  if ((gate<0) || (constants::numGatesPerTunnel<=gate)) {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "gate argument out of range");
    flag = false;
  }
  return flag;
}

bool MessageHandler::checkLedArg(int led) {
  bool flag = true;
  if ((led<0) || (constants::numLedsPerTunnel<=led)) {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "led argument out of range");
    flag = false;
  }
  return flag;
}

bool MessageHandler::checkServoAngleArg(int angle) {
  bool flag = true;
  if ((angle<constants::gateMinServoAngle) || (constants::gateMaxServoAngle<angle)) {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "servo angle argument out of range");
    flag = false;
  }
  return flag;
}

bool MessageHandler::checkLedDutyCycleArg(int duty_cycle) {
  bool flag = true;
  if ((duty_cycle<constants::ledMinDutyCycle) || (constants::ledMaxDutyCycle<duty_cycle)) {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", "led duty cycle argument out of range");
    flag = false;
  }
  return flag;
}

void MessageHandler::systemCmdRsp(bool flag) {
  if (flag) {
    dprint.addIntItem("status", rspSuccess);
  }
  else {
    dprint.addIntItem("status", rspError);
    dprint.addStrItem("err_msg", systemState.errMsg);
  }
}

void MessageHandler::handleGetRspCodes() {
  dprint.addIntItem("status", rspSuccess);
  dprint.addIntItem("rsp_success",rspSuccess);
  dprint.addIntItem("rsp_error", rspError);
}

void MessageHandler::handleSetSerialNumber() {
  if (!checkNumberOfArgs(2)) {return;}
  int serialNumber = readInt(1);
  if (checkSerialNumberArg(serialNumber)) {
    systemCmdRsp(savedVariables.setSerialNumber(serialNumber));
  }
  dprint.addIntItem("status", rspSuccess);
}

void MessageHandler::handleSetGateOpenServoAngle() {
  if (!checkNumberOfArgs(2)) {return;}
  int angle = readInt(1);
  if (checkServoAngleArg(angle)) {
    systemCmdRsp(savedVariables.setGateOpenServoAngle(angle));
  }
  dprint.addIntItem("status", rspSuccess);
}

void MessageHandler::handleSetGateCloseServoAngle() {
  if (!checkNumberOfArgs(2)) {return;}
  int angle = readInt(1);
  if (checkServoAngleArg(angle)) {
    systemCmdRsp(savedVariables.setGateCloseServoAngle(angle));
  }
  dprint.addIntItem("status", rspSuccess);
}

void MessageHandler::handleSetLedOnDutyCycle() {
  if (!checkNumberOfArgs(2)) {return;}
  int duty_cycle = readInt(1);
  if (checkLedDutyCycleArg(duty_cycle)) {
    systemCmdRsp(savedVariables.setLedOnDutyCycle(duty_cycle));
  }
  dprint.addIntItem("status", rspSuccess);
}

void MessageHandler::handleOpenGate() {
  if (!checkNumberOfArgs(3)) {return;}
  int tunnel = readInt(1);
  int gate = readInt(2);
  if (checkTunnelArg(tunnel) && checkGateArg(gate)) {
    systemCmdRsp(systemState.openGate(tunnel,gate));
  }
}

void MessageHandler::handleCloseGate() {
  if (!checkNumberOfArgs(3)) {return;}
  int tunnel = readInt(1);
  int gate = readInt(2);
  if (checkTunnelArg(tunnel) && checkGateArg(gate)) {
    systemCmdRsp(systemState.closeGate(tunnel,gate));
  }
}

void MessageHandler::handleOpenAllGatesByGate() {
  if (!checkNumberOfArgs(2)) {return;}
  int gate = readInt(1);
  if (checkGateArg(gate)) {
    systemCmdRsp(systemState.openAllGatesByGate(gate));
  }
}

void MessageHandler::handleCloseAllGatesByGate() {
  if (!checkNumberOfArgs(2)) {return;}
  int gate = readInt(1);
  if (checkGateArg(gate)) {
    systemCmdRsp(systemState.closeAllGatesByGate(gate));
  }
}

void MessageHandler::handleOpenAllGatesByTunnel() {
  if (!checkNumberOfArgs(2)) {return;}
  int tunnel = readInt(1);
  if (checkTunnelArg(tunnel)) {
    systemCmdRsp(systemState.openAllGatesByTunnel(tunnel));
  }
}

void MessageHandler::handleCloseAllGatesByTunnel() {
  if (!checkNumberOfArgs(2)) {return;}
  int tunnel = readInt(1);
  if (checkTunnelArg(tunnel)) {
    systemCmdRsp(systemState.closeAllGatesByTunnel(tunnel));
  }
}

void MessageHandler::handleOpenAllGates() {
  systemCmdRsp(systemState.openAllGates());
}

void MessageHandler::handleCloseAllGates() {
  systemCmdRsp(systemState.closeAllGates());
}

void MessageHandler::handleSetGateServoAngle() {
  if (!checkNumberOfArgs(4)) {return;}
  int tunnel = readInt(1);
  int gate = readInt(2);
  int angle = readInt(3);
  if (checkTunnelArg(tunnel) && checkGateArg(gate) && checkServoAngleArg(angle)) {
    systemCmdRsp(systemState.setGateServoAngle(tunnel,gate,angle));
  }
}

void MessageHandler::handleSetAllGatesServoAngleByGate() {
  if (!checkNumberOfArgs(3)) {return;}
  int gate = readInt(1);
  int angle = readInt(2);
  if (checkGateArg(gate) && checkServoAngleArg(angle)) {
    systemCmdRsp(systemState.setAllGatesServoAngleByGate(gate,angle));
  }
}

void MessageHandler::handleSetAllGatesServoAngleByTunnel() {
  if (!checkNumberOfArgs(3)) {return;}
  int tunnel = readInt(1);
  int angle = readInt(2);
  if (checkTunnelArg(tunnel) && checkServoAngleArg(angle)) {
    systemCmdRsp(systemState.setAllGatesServoAngleByTunnel(tunnel,angle));
  }
}

void MessageHandler::handleSetAllGatesServoAngle() {
  if (!checkNumberOfArgs(2)) {return;}
  int angle = readInt(1);
  if (checkServoAngleArg(angle)) {
    systemCmdRsp(systemState.setAllGatesServoAngle(angle));
  }
}

void MessageHandler::handleTurnOnAllLeds() {
  systemCmdRsp(systemState.turnOnAllLeds());
}

void MessageHandler::handleTurnOffAllLeds() {
  systemCmdRsp(systemState.turnOffAllLeds());
}

void MessageHandler::handleTurnOnAllLedsByLed() {
  if (!checkNumberOfArgs(2)) {return;}
  int led = readInt(1);
  if (checkLedArg(led)) {
    systemCmdRsp(systemState.turnOnAllLedsByLed(led));
  }
}

void MessageHandler::handleTurnOffAllLedsByLed() {
  if (!checkNumberOfArgs(2)) {return;}
  int led = readInt(1);
  if (checkLedArg(led)) {
    systemCmdRsp(systemState.turnOffAllLedsByLed(led));
  }
}

void MessageHandler::handleTurnOnAllLedsByTunnel() {
  if (!checkNumberOfArgs(2)) {return;}
  int tunnel = readInt(1);
  if (checkTunnelArg(tunnel)) {
    systemCmdRsp(systemState.turnOnAllLedsByTunnel(tunnel));
  }
}

void MessageHandler::handleTurnOffAllLedsByTunnel() {
  if (!checkNumberOfArgs(2)) {return;}
  int tunnel = readInt(1);
  if (checkTunnelArg(tunnel)) {
    systemCmdRsp(systemState.turnOffAllLedsByTunnel(tunnel));
  }
}

void MessageHandler::handleTurnOnLed() {
  if (!checkNumberOfArgs(3)) {return;}
  int tunnel = readInt(1);
  int led = readInt(2);
  if (checkTunnelArg(tunnel) && checkLedArg(led)) {
    systemCmdRsp(systemState.turnOnLed(tunnel,led));
  }
}

void MessageHandler::handleTurnOffLed() {
  if (!checkNumberOfArgs(3)) {return;}
  int tunnel = readInt(1);
  int led = readInt(2);
  if (checkTunnelArg(tunnel) && checkLedArg(led)) {
    systemCmdRsp(systemState.turnOffLed(tunnel,led));
  }
}

void MessageHandler::handleSetLedDutyCycle() {
  if (!checkNumberOfArgs(4)) {return;}
  int tunnel = readInt(1);
  int led = readInt(2);
  int duty_cycle = readInt(3);
  if (checkTunnelArg(tunnel) && checkLedArg(led) && checkLedDutyCycleArg(duty_cycle)) {
    systemCmdRsp(systemState.setLedDutyCycle(tunnel,led,duty_cycle));
  }
}

void MessageHandler::handleSetAllLedsDutyCycleByLed() {
  if (!checkNumberOfArgs(3)) {return;}
  int led = readInt(1);
  int duty_cycle = readInt(2);
  if (checkLedArg(led) && checkLedDutyCycleArg(duty_cycle)) {
    systemCmdRsp(systemState.setAllLedsDutyCycleByLed(led,duty_cycle));
  }
}

void MessageHandler::handleSetAllLedsDutyCycleByTunnel() {
  if (!checkNumberOfArgs(3)) {return;}
  int tunnel = readInt(1);
  int duty_cycle = readInt(2);
  if (checkTunnelArg(tunnel) && checkLedDutyCycleArg(duty_cycle)) {
    systemCmdRsp(systemState.setAllLedsDutyCycleByTunnel(tunnel,duty_cycle));
  }
}

void MessageHandler::handleSetAllLedsDutyCycle() {
  if (!checkNumberOfArgs(2)) {return;}
  int duty_cycle = readInt(1);
  if (checkLedDutyCycleArg(duty_cycle)) {
    systemCmdRsp(systemState.setAllLedsDutyCycle(duty_cycle));
  }
}

// void MessageHandler::handleMoveToPosition() {
//   Array<float,constants::numTunnel> pos;
//   if (!checkNumberOfArgs(constants::numTunnel+1)) {return;}
//   for (int i=0; i<constants::numTunnel; i++) {
//     pos[i] = readFloat(i+1);
//   }
//   systemCmdRsp(systemState.moveToPosition(pos));
// }

// -------------------------------------------------


void MessageHandler::handleDebug() {
  char name[20];
  dprint.addIntItem("status", rspSuccess);
}


MessageHandler messageHandler;
