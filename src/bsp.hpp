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

  static void powerOffAll();
  static void powerOnAll();
  static void setupPrism(uint8_t prism_address);
  static bool communicating(uint8_t prism_address);
  static void setupParametersAndEnable(uint8_t prism_address);
  static void beginHome(uint8_t prism_address);
  static void endHome(uint8_t prism_address);
  static bool homed(uint8_t prism_address);
  static void writeTargetPosition(uint8_t prism_address, uint16_t position_mm);
  static void pause(uint8_t prism_address);
  static void resume(uint8_t prism_address);
  static int16_t readActualPosition(uint8_t prism_address);

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
