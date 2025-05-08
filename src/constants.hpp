#ifndef CLUSTER_CONTROLLER_CONSTANTS_HPP
#define CLUSTER_CONTROLLER_CONSTANTS_HPP


namespace CC
{
namespace constants
{
constexpr uint32_t ticks_per_second = 1000;

constexpr uint32_t watchdog_delay_ms = 2000;

// Cluster
constexpr uint32_t cluster_timer_delay_s = 2;
constexpr uint32_t cluster_timer_frequency_hz = 10;

// Prism
constexpr uint32_t home_delay_s = 2;
constexpr uint32_t home_delay_count = cluster_timer_frequency_hz * home_delay_s;

// Serial Communication Interface
constexpr uint32_t serial_timer_delay_s = 1;
constexpr uint32_t serial_timer_frequency_hz = 50;
constexpr uint32_t serial_baud_rate = 115200;
constexpr uint16_t serial_timeout = 100;

// Ethernet Communication Interface
constexpr uint32_t ethernet_timer_delay_s = 1;
constexpr uint32_t ethernet_timer_frequency_hz = 100;
constexpr uint8_t mac_address_size = 6;
constexpr uint32_t ethernet_server_port = 7777;

// Protocol
constexpr uint8_t protocol_version = 0x02;
constexpr uint8_t command_length_min = 3;
constexpr uint8_t response_length_index = 1;
constexpr uint8_t response_header_size = 3;

// Commands
constexpr uint16_t string_command_length_max = 512;
constexpr byte first_command_byte_max_value_binary = 0x23;
constexpr byte byte_count_per_command_max = 16;
constexpr char command_termination_character = '\n';

// Response
constexpr uint16_t string_response_length_max = 512;
constexpr uint16_t byte_count_per_response_max = 32;
constexpr uint8_t error_response = 0xEE;
constexpr uint32_t check_communication_response = 0x12345678;

// Log
constexpr uint16_t string_log_length_max = 512;

// Conversions
constexpr uint32_t milliseconds_per_second = 1000;
constexpr uint32_t microseconds_per_second = 1000000;
constexpr uint8_t bit_count_per_byte = 8;

// Cluster Addresses
constexpr uint8_t cluster_address_min = 0;
constexpr uint8_t cluster_address_max = 255;

// Beep
constexpr uint16_t beep_frequency_min = 1000;
constexpr uint16_t beep_frequency_max = 10000;


// Prisms
constexpr uint8_t prism_count_max = 7;

}
}
#endif
