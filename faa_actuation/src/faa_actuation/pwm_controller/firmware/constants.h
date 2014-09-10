#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

namespace constants {
  enum {numTunnels=6};
  enum {numGatesPerTunnel=3};
  enum {numLedsPerTunnel=2};

  // Communications parameters
  extern const int baudrate;

  // Device parameters
  extern const int modelNumber;
  extern const int serialNumberMin;
  extern const int serialNumberMax;
  extern const int firmwareNumber;

  // Pin assignment
  extern const int gateServoPinArray[numTunnels][numGatesPerTunnel];
  extern const int ledChannelArray[numTunnels][numLedsPerTunnel];

  // Tunnel parameters
  extern const int gateMinServoAngle;
  extern const int gateMaxServoAngle;
  extern const int ledMinValue;
  extern const int ledMaxValue;
  extern const int ledMinDutyCycle;
  extern const int ledMaxDutyCycle;
  extern const int gateDelay;
}
#endif
