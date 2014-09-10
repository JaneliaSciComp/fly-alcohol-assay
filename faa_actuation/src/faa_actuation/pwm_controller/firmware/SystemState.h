#ifndef _SYSTEM_STATE_H_
#define _SYSTEM_STATE_H_
#include "constants.h"
#include "Array.h"
#include "Tunnel.h"

enum {SYS_ERR_BUF_SZ=50};

class SystemState {

 public:
  SystemState();

  void setErrMsg(char *);
  char errMsg[SYS_ERR_BUF_SZ];

  void initialize();

  bool openGate(int tunnel, int gate);
  bool closeGate(int tunnel, int gate);

  bool openAllGatesByGate(int gate);
  bool closeAllGatesByGate(int gate);

  bool openAllGatesByTunnel(int tunnel);
  bool closeAllGatesByTunnel(int tunnel);

  bool openAllGates();
  bool closeAllGates();

  bool setGateServoAngle(int tunnel, int gate, int angle);
  bool setAllGatesServoAngleByGate(int gate, int angle);
  bool setAllGatesServoAngleByTunnel(int tunnel, int angle);
  bool setAllGatesServoAngle(int angle);

  bool turnOnLed(int tunnel, int led);
  bool turnOffLed(int tunnel, int led);

  bool turnOnAllLedsByLed(int led);
  bool turnOffAllLedsByLed(int led);

  bool turnOnAllLedsByTunnel(int tunnel);
  bool turnOffAllLedsByTunnel(int tunnel);

  bool turnOnAllLeds();
  bool turnOffAllLeds();

  bool setLedDutyCycle(int tunnel, int led, int duty_cycle);
  bool setAllLedsDutyCycleByLed(int led, int duty_cycle);
  bool setAllLedsDutyCycleByTunnel(int tunnel, int duty_cycle);
  bool setAllLedsDutyCycle(int duty_cycle);


 private:
  Tunnel _tunnelArray[constants::numTunnels];

  bool checkTunnelArg(int tunnel);
  bool checkDutyCycleArg(int duty_cycle);

};

extern SystemState systemState;

#endif
