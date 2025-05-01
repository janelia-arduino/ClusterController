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

  static void beep(uint16_t duration_ms);

  static void powerOffAllPrisms();
  static void powerOnAllPrisms();
  static void setupPrism(uint8_t prism_address);

  static bool beginSerial();
  static bool pollSerialCommand();
  static uint8_t readSerialByte();
  static void readSerialStringCommand(char * command_str, char first_char);
  static void writeSerialStringResponse(char * response);

  static bool initializeEthernet();
  static void pollEthernet();
  static bool createEthernetServerConnection();
  static void writeEthernetBinaryResponse(void * connection, uint8_t response[CC::constants::byte_count_per_response_max], uint8_t response_byte_count);
};

#endif // BSP_HPP
