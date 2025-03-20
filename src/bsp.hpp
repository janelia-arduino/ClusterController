#ifndef BSP_HPP
#define BSP_HPP
#include "constants.hpp"

class BSP {
public:
  static void init();

  static void ledOff();
  static void ledOn();

  static void initializeWatchdog();
  static void feedWatchdog();

  static void initializeCluster();

  static uint8_t readClusterAddress();

  static void powerOff();
  static void powerOn();

  static bool beginSerial();
  static bool pollSerialCommand();
  static void readSerialStringCommand(char * command_str);
  static void writeSerialStringResponse(char * response);

  static void initializeEthernet();
  static bool beginEthernet();
  static bool beginEthernetServer();
  static bool pollEthernetCommand();
};

#endif // BSP_HPP
