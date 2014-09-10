#include <Streaming.h>
#include <Servo.h>
#include <EEPROM.h>
#include "Tlc5940.h"
#include "Tunnel.h"
#include "SerialReceiver.h"
#include "DictPrinter.h"
#include "Array.h"
#include "MessageHandler.h"
#include "SystemState.h"
#include "constants.h"

void setup() {
    Serial.begin(constants::baudrate);
    systemState.initialize();
}

void loop() { 
    messageHandler.processMsg(); 
}


