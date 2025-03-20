#ifndef CLUSTER_CONTROLLER_CONSTANTS_HPP
#define CLUSTER_CONTROLLER_CONSTANTS_HPP

#include <SPI.h>


namespace CC
{
namespace constants
{
constexpr uint32_t ticks_per_second = 1000;

constexpr uint32_t watchdog_delay_ms = 500;

// Serial Communication Interface
constexpr uint32_t serial_baud_rate = 115200;
constexpr uint16_t serial_timeout = 100;

// Ethernet Communication Interface
constexpr uint8_t mac_address_size = 6;
constexpr uint32_t ethernet_server_port = 7777;

// SPI Settings
constexpr uint8_t spi_bit_order = MSBFIRST;
constexpr uint8_t spi_data_mode = SPI_MODE0;

// Conversions
constexpr uint32_t milliseconds_per_second = 1000;
constexpr uint32_t microseconds_per_second = 1000000;

}
}
#endif
