#include "constants.h"
namespace constants {
  // Communications parameters
  const int baudrate = 9600;

  // Device parameters
  const int modelNumber = 1154;
  const int serialNumberMin = 0;
  const int serialNumberMax = 255;
  const int firmwareNumber = 1;

  // Pin assignment
  const int gateServoPinArray[numTunnels][numGatesPerTunnel] =
    {
      { 22, 23, 24 }, // tunnel 0
      { 25, 26, 27 }, // tunnel 1
      { 28, 29, 30 }, // tunnel 2
      { 31, 32, 33 }, // tunnel 3
      { 34, 35, 36 }, // tunnel 4
      { 37, 38, 39 }, // tunnel 5
    };
  const int ledChannelArray[numTunnels][numLedsPerTunnel] =
    {
      {  0,  1 }, // tunnel 0
      {  2,  3 }, // tunnel 1
      {  4,  5 }, // tunnel 2
      {  6,  7 }, // tunnel 3
      {  8,  9 }, // tunnel 4
      { 10, 11 }, // tunnel 5
    };

  // Tunnel parameters
  const int gateMinServoAngle = 20;
  const int gateMaxServoAngle = 165;
  const int ledMinValue = 0;
  const int ledMaxValue = 4095;
  const int ledMinDutyCycle = 0;
  const int ledMaxDutyCycle = 100;
  const int gateDelay = 20;

}
