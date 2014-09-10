#ifndef _MESSAGE_HANDER_H_
#define _MESSAGE_HANDER_H_
#include <SerialReceiver.h>
#include "DictPrinter.h"
#include "constants.h"
#include "SavedVariables.h"

class MessageHandler : public SerialReceiver {

 public:
  void processMsg();

 private:
  DictPrinter dprint;
  void msgSwitchYard();
  bool checkNumberOfArgs(int num);
  bool checkSerialNumberArg(int serial_number);
  bool checkTunnelArg(int tunnel);
  bool checkGateArg(int gate);
  bool checkLedArg(int led);
  bool checkServoAngleArg(int angle);
  bool checkLedDutyCycleArg(int duty_cycle);
  void systemCmdRsp(bool flag);

  void handleGetDevInfo();
  void handleGetCmds();
  void handleGetRspCodes();
  void handleSetSerialNumber();
  void handleSetGateOpenServoAngle();
  void handleSetGateCloseServoAngle();
  void handleSetLedOnDutyCycle();

  void handleOpenGate();
  void handleCloseGate();
  void handleOpenAllGatesByGate();
  void handleCloseAllGatesByGate();
  void handleOpenAllGatesByTunnel();
  void handleCloseAllGatesByTunnel();
  void handleOpenAllGates();
  void handleCloseAllGates();
  void handleSetGateServoAngle();
  void handleSetAllGatesServoAngleByGate();
  void handleSetAllGatesServoAngleByTunnel();
  void handleSetAllGatesServoAngle();

  void handleTurnOnAllLeds();
  void handleTurnOffAllLeds();
  void handleTurnOnAllLedsByLed();
  void handleTurnOffAllLedsByLed();
  void handleTurnOnAllLedsByTunnel();
  void handleTurnOffAllLedsByTunnel();
  void handleTurnOnLed();
  void handleTurnOffLed();
  void handleSetLedDutyCycle();
  void handleSetAllLedsDutyCycleByLed();
  void handleSetAllLedsDutyCycleByTunnel();
  void handleSetAllLedsDutyCycle();


  // Development
  void handleGetTimerCount();
  void handleDebug();
};

extern MessageHandler messageHandler;
#endif
