// Tunnel.h
#ifndef Tunnel_h
#define Tunnel_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif
#include "constants.h"
#include <Servo.h>
#include "Tlc5940.h"
#include "SavedVariables.h"


class Tunnel {

 public:

  Tunnel();
  Tunnel(int tunnel);

  bool openGate(int gate);
  bool closeGate(int gate);

  bool openAllGates();
  bool closeAllGates();

  bool setGateServoAngle(int gate, int angle);
  bool setAllGatesServoAngle(int angle);

  bool turnOnLed(int led);
  bool turnOffLed(int led);

  bool turnOnAllLeds();
  bool turnOffAllLeds();

  bool setLedDutyCycle(int led, int duty_cycle);
  bool setAllLedsDutyCycle(int duty_cycle);

 private:

  int _tunnel;
  Servo _gateServoArray[constants::numGatesPerTunnel];

  bool checkGateArg(int gate);
  bool checkLedArg(int led);
  bool checkLedDutyCycleArg(int duty_cycle);
  bool checkServoAngleArg(int angle);
  bool checkLedValueArg(int value);

  void initialize();

};

inline int convertDutyCycleToLedValue(int duty_cycle) {return map(duty_cycle,
                                                                  constants::ledMinDutyCycle,
                                                                  constants::ledMaxDutyCycle,
                                                                  constants::ledMinValue,
                                                                  constants::ledMaxValue);}

#endif
