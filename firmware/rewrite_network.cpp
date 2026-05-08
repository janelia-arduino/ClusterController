#include "rewrite_network.hpp"

#include "rewrite_cluster_address.hpp"
#include "rewrite_bsp.hpp"
#include "rewrite_prism.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <W5500lwIP.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

#include <cstring>

namespace rewrite_network
{
namespace
{

constexpr pin_size_t ethernet_spi_rx_pin = 16;
constexpr pin_size_t ethernet_spi_csn_pin = 17;
constexpr pin_size_t ethernet_spi_sck_pin = 18;
constexpr pin_size_t ethernet_spi_tx_pin = 19;
constexpr pin_size_t ethernet_reset_pin = 20;
constexpr pin_size_t ethernet_int_pin = 21;
constexpr uint16_t server_port = 7777;
constexpr uint8_t protocol_version = 0x06;
constexpr uint8_t error_response = 0xEE;
constexpr uint8_t read_cluster_address_cmd = 0x01;
constexpr uint8_t communicating_cluster_cmd = 0x02;
constexpr uint8_t reset_cluster_cmd = 0x03;
constexpr uint8_t beep_cluster_cmd = 0x04;
constexpr uint8_t led_off_cluster_cmd = 0x05;
constexpr uint8_t led_on_cluster_cmd = 0x06;
constexpr uint8_t power_off_cluster_cmd = 0x07;
constexpr uint8_t power_on_cluster_cmd = 0x08;
constexpr uint8_t home_prism_cmd = 0x09;
constexpr uint8_t home_cluster_cmd = 0x0A;
constexpr uint8_t write_target_prism_cmd = 0x0C;
constexpr uint8_t write_targets_cluster_cmd = 0x0D;
constexpr uint8_t pause_prism_cmd = 0x0E;
constexpr uint8_t pause_cluster_cmd = 0x0F;
constexpr uint8_t resume_prism_cmd = 0x10;
constexpr uint8_t resume_cluster_cmd = 0x11;
constexpr uint8_t homed_cluster_cmd = 0x0B;
constexpr uint8_t read_positions_cluster_cmd = 0x12;
constexpr uint8_t write_run_current_cluster_cmd = 0x13;
constexpr uint8_t read_run_current_cluster_cmd = 0x14;
constexpr uint8_t write_controller_parameters_cluster_cmd = 0x15;
constexpr uint8_t read_controller_parameters_cluster_cmd = 0x16;
constexpr uint8_t write_double_target_prism_cmd = 0x17;
constexpr uint8_t write_double_targets_cluster_cmd = 0x18;
constexpr uint8_t read_home_outcomes_cluster_cmd = 0x19;
constexpr uint8_t read_prism_diagnostics_cluster_cmd = 0x1A;
constexpr uint8_t clear_prism_diagnostics_cluster_cmd = 0x1B;
constexpr uint8_t recovery_home_prism_cmd = 0x1C;
constexpr uint8_t recovery_home_cluster_cmd = 0x1D;
constexpr uint8_t confirm_home_prism_cmd = 0x1E;
constexpr uint8_t confirm_home_cluster_cmd = 0x1F;
constexpr uint8_t reboot_bootloader_cluster_cmd = 0x20;
constexpr uint32_t check_communication_response = 0x12345678;
constexpr size_t command_buffer_size = 32;
constexpr size_t response_buffer_size = 64;
constexpr uint32_t prism_power_stabilize_delay_ms = 2000;
constexpr uint8_t run_current_default = 75;
constexpr uint8_t start_velocity_default = 10;
constexpr uint8_t stop_velocity_default = 10;
constexpr uint8_t first_velocity_default = 40;
constexpr uint8_t max_velocity_default = 40;
constexpr uint8_t first_acceleration_default = 120;
constexpr uint8_t max_acceleration_default = 80;
constexpr uint8_t max_deceleration_default = 80;
constexpr uint8_t first_deceleration_default = 120;

uint8_t run_current_percent = run_current_default;
rewrite_prism::ControllerParameters controller_parameters = {
    start_velocity_default,  stop_velocity_default, first_velocity_default,
    max_velocity_default,    first_acceleration_default,
    max_acceleration_default, max_deceleration_default,
    first_deceleration_default};

SPIClassRP2040 &ethernet_spi = SPI;
Wiznet5500lwIP eth(ethernet_spi_csn_pin, ethernet_spi, ethernet_int_pin);
WiFiServer server(server_port);
WiFiClient active_client;
bool initialized = false;
bool server_started = false;
bool prism_setup_pending = false;
bool bootloader_reboot_pending = false;
uint32_t prism_setup_start_ms = 0;
uint8_t command_buffer[command_buffer_size];
uint8_t response_buffer[response_buffer_size];

size_t build_error_response()
{
  response_buffer[0] = protocol_version;
  response_buffer[1] = 3;
  response_buffer[2] = error_response;
  return 3;
}

size_t process_command(const uint8_t *command, const size_t command_size)
{
  if (command_size < 3) {
    return build_error_response();
  }

  response_buffer[0] = protocol_version;
  response_buffer[1] = 0;

  if (command[0] != protocol_version || command[1] != command_size) {
    return build_error_response();
  }

  const uint8_t command_number = command[2];
  size_t response_size = 3;
  response_buffer[2] = command_number;

  switch (command_number) {
    case read_cluster_address_cmd:
      response_buffer[response_size++] = rewrite_cluster_address::read();
      break;

    case communicating_cluster_cmd:
      std::memcpy(response_buffer + response_size,
                  &check_communication_response,
                  sizeof(check_communication_response));
      response_size += sizeof(check_communication_response);
      break;

    case reset_cluster_cmd:
      rewrite_bsp::set_cluster_power(false);
      rewrite_prism::shutdown();
      prism_setup_pending = false;
      break;

    case beep_cluster_cmd:
    {
      if (command_size != 5) {
        return build_error_response();
      }
      uint16_t duration_ms = 0;
      std::memcpy(&duration_ms, command + 3, sizeof(duration_ms));
      rewrite_bsp::beep(duration_ms);
      break;
    }

    case led_off_cluster_cmd:
      rewrite_bsp::set_command_led(false);
      break;

    case led_on_cluster_cmd:
      rewrite_bsp::set_command_led(true);
      break;

    case power_off_cluster_cmd:
      rewrite_bsp::set_cluster_power(false);
      rewrite_prism::shutdown();
      prism_setup_pending = false;
      break;

    case power_on_cluster_cmd:
      rewrite_bsp::set_cluster_power(true);
      prism_setup_start_ms = millis();
      prism_setup_pending = true;
      break;

    case home_cluster_cmd:
    {
      if (command_size != 8) {
        return build_error_response();
      }
      rewrite_prism::HomeParameters home_parameters{};
      std::memcpy(&home_parameters.travel_limit, command + 3,
                  sizeof(home_parameters.travel_limit));
      home_parameters.max_velocity = command[5];
      home_parameters.run_current = command[6];
      home_parameters.stall_threshold = static_cast<int8_t>(command[7]);
      home_parameters =
          rewrite_prism::clamp_home_parameters(home_parameters, false);
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::begin_home(prism_address, home_parameters);
      }
      break;
    }

    case home_prism_cmd:
    {
      if (command_size != 9) {
        return build_error_response();
      }
      const uint8_t prism_address = command[3];
      rewrite_prism::HomeParameters home_parameters{};
      std::memcpy(&home_parameters.travel_limit, command + 4,
                  sizeof(home_parameters.travel_limit));
      home_parameters.max_velocity = command[6];
      home_parameters.run_current = command[7];
      home_parameters.stall_threshold = static_cast<int8_t>(command[8]);
      home_parameters =
          rewrite_prism::clamp_home_parameters(home_parameters, false);
      rewrite_prism::begin_home(prism_address, home_parameters);
      response_buffer[response_size++] = prism_address;
      break;
    }

    case recovery_home_cluster_cmd:
    {
      if (command_size != 8) {
        return build_error_response();
      }
      rewrite_prism::HomeParameters home_parameters{};
      std::memcpy(&home_parameters.travel_limit, command + 3,
                  sizeof(home_parameters.travel_limit));
      home_parameters.max_velocity = command[5];
      home_parameters.run_current = command[6];
      home_parameters.stall_threshold = static_cast<int8_t>(command[7]);
      home_parameters =
          rewrite_prism::clamp_home_parameters(home_parameters, true);
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::begin_recovery_home(prism_address, home_parameters);
      }
      break;
    }

    case recovery_home_prism_cmd:
    {
      if (command_size != 9) {
        return build_error_response();
      }
      const uint8_t prism_address = command[3];
      rewrite_prism::HomeParameters home_parameters{};
      std::memcpy(&home_parameters.travel_limit, command + 4,
                  sizeof(home_parameters.travel_limit));
      home_parameters.max_velocity = command[6];
      home_parameters.run_current = command[7];
      home_parameters.stall_threshold = static_cast<int8_t>(command[8]);
      home_parameters =
          rewrite_prism::clamp_home_parameters(home_parameters, true);
      rewrite_prism::begin_recovery_home(prism_address, home_parameters);
      response_buffer[response_size++] = prism_address;
      break;
    }

    case confirm_home_cluster_cmd:
      if (command_size != 3) {
        return build_error_response();
      }
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::confirm_home(prism_address);
      }
      break;

    case confirm_home_prism_cmd:
    {
      if (command_size != 4) {
        return build_error_response();
      }
      const uint8_t prism_address = command[3];
      rewrite_prism::confirm_home(prism_address);
      response_buffer[response_size++] = prism_address;
      break;
    }

    case reboot_bootloader_cluster_cmd:
      if (command_size != 3) {
        return build_error_response();
      }
      bootloader_reboot_pending = true;
      break;

    case write_target_prism_cmd:
    {
      if (command_size != 6) {
        return build_error_response();
      }
      const uint8_t prism_address = command[3];
      int16_t position_mm = 0;
      std::memcpy(&position_mm, command + 4, sizeof(position_mm));
      rewrite_prism::write_target(prism_address, position_mm);
      response_buffer[response_size++] = prism_address;
      break;
    }

    case write_targets_cluster_cmd:
      if (command_size != 17) {
        return build_error_response();
      }
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        int16_t position_mm = 0;
        std::memcpy(&position_mm, command + 3 + prism_address * sizeof(position_mm),
                    sizeof(position_mm));
        rewrite_prism::write_target(prism_address, position_mm);
      }
      break;

    case write_double_target_prism_cmd:
    {
      if (command_size != 8) {
        return build_error_response();
      }
      const uint8_t prism_address = command[3];
      int16_t first_position_mm = 0;
      int16_t second_position_mm = 0;
      std::memcpy(&first_position_mm, command + 4, sizeof(first_position_mm));
      std::memcpy(&second_position_mm, command + 6, sizeof(second_position_mm));
      rewrite_prism::write_target(prism_address, first_position_mm);
      rewrite_prism::write_target(prism_address, second_position_mm);
      response_buffer[response_size++] = prism_address;
      break;
    }

    case write_double_targets_cluster_cmd:
      if (command_size != 31) {
        return build_error_response();
      }
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        int16_t first_position_mm = 0;
        int16_t second_position_mm = 0;
        const size_t base_offset = 3 + prism_address * 2 * sizeof(first_position_mm);
        std::memcpy(&first_position_mm, command + base_offset,
                    sizeof(first_position_mm));
        std::memcpy(&second_position_mm,
                    command + base_offset + sizeof(first_position_mm),
                    sizeof(second_position_mm));
        rewrite_prism::write_target(prism_address, first_position_mm);
        rewrite_prism::write_target(prism_address, second_position_mm);
      }
      break;

    case pause_prism_cmd:
    {
      if (command_size != 4) {
        return build_error_response();
      }
      const uint8_t prism_address = command[3];
      rewrite_prism::pause(prism_address);
      response_buffer[response_size++] = prism_address;
      break;
    }

    case pause_cluster_cmd:
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::pause(prism_address);
      }
      break;

    case resume_prism_cmd:
    {
      if (command_size != 4) {
        return build_error_response();
      }
      const uint8_t prism_address = command[3];
      rewrite_prism::resume(prism_address);
      response_buffer[response_size++] = prism_address;
      break;
    }

    case resume_cluster_cmd:
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::resume(prism_address);
      }
      break;

    case homed_cluster_cmd:
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        response_buffer[response_size++] =
            rewrite_prism::homed(prism_address) ? 1 : 0;
      }
      break;

    case read_positions_cluster_cmd:
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        const int16_t position_mm =
            rewrite_prism::read_position_mm(prism_address);
        std::memcpy(response_buffer + response_size,
                    &position_mm,
                    sizeof(position_mm));
        response_size += sizeof(position_mm);
      }
      break;

    case write_run_current_cluster_cmd:
      if (command_size != 4) {
        return build_error_response();
      }
      run_current_percent = rewrite_prism::clamp_run_current_percent(command[3]);
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::write_run_current(prism_address, run_current_percent);
      }
      break;

    case read_run_current_cluster_cmd:
      response_buffer[response_size++] = run_current_percent;
      break;

    case write_controller_parameters_cluster_cmd:
      if (command_size != 11) {
        return build_error_response();
      }
      controller_parameters.start_velocity = command[3];
      controller_parameters.stop_velocity = command[4];
      controller_parameters.first_velocity = command[5];
      controller_parameters.max_velocity = command[6];
      controller_parameters.first_acceleration = command[7];
      controller_parameters.max_acceleration = command[8];
      controller_parameters.max_deceleration = command[9];
      controller_parameters.first_deceleration = command[10];
      controller_parameters =
          rewrite_prism::clamp_controller_parameters(controller_parameters);
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::write_controller_parameters(prism_address,
                                                   controller_parameters);
      }
      break;

    case read_controller_parameters_cluster_cmd:
      response_buffer[response_size++] = controller_parameters.start_velocity;
      response_buffer[response_size++] = controller_parameters.stop_velocity;
      response_buffer[response_size++] = controller_parameters.first_velocity;
      response_buffer[response_size++] = controller_parameters.max_velocity;
      response_buffer[response_size++] =
          controller_parameters.first_acceleration;
      response_buffer[response_size++] =
          controller_parameters.max_acceleration;
      response_buffer[response_size++] =
          controller_parameters.max_deceleration;
      response_buffer[response_size++] =
          controller_parameters.first_deceleration;
      break;

    case read_home_outcomes_cluster_cmd:
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        response_buffer[response_size++] =
            rewrite_prism::home_outcome(prism_address);
      }
      break;

    case read_prism_diagnostics_cluster_cmd:
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        const rewrite_prism::PrismDiagnostics diagnostics =
            rewrite_prism::read_diagnostics(prism_address);
        response_buffer[response_size++] = diagnostics.health_flags;
        response_buffer[response_size++] = diagnostics.driver_flags;
        std::memcpy(response_buffer + response_size,
                    &diagnostics.stall_guard_result,
                    sizeof(diagnostics.stall_guard_result));
        response_size += sizeof(diagnostics.stall_guard_result);
        response_buffer[response_size++] = diagnostics.current_scale;
        std::memcpy(response_buffer + response_size,
                    &diagnostics.last_home_travel_mm,
                    sizeof(diagnostics.last_home_travel_mm));
        response_size += sizeof(diagnostics.last_home_travel_mm);
      }
      break;

    case clear_prism_diagnostics_cluster_cmd:
      for (size_t prism_address = 0; prism_address < rewrite_prism::prism_count;
           ++prism_address) {
        rewrite_prism::clear_diagnostics(prism_address);
      }
      break;

    default:
      return build_error_response();
  }

  response_buffer[1] = response_size;
  return response_size;
}

void hardware_reset()
{
  digitalWrite(ethernet_reset_pin, LOW);
  delay(2);
  digitalWrite(ethernet_reset_pin, HIGH);
  delay(150);
}

} // namespace

void setup()
{
  pinMode(ethernet_spi_csn_pin, OUTPUT);
  digitalWrite(ethernet_spi_csn_pin, HIGH);

  pinMode(ethernet_reset_pin, OUTPUT);
  pinMode(ethernet_int_pin, INPUT_PULLUP);

  ethernet_spi.setRX(ethernet_spi_rx_pin);
  ethernet_spi.setCS(ethernet_spi_csn_pin);
  ethernet_spi.setSCK(ethernet_spi_sck_pin);
  ethernet_spi.setTX(ethernet_spi_tx_pin);

  rewrite_cluster_address::setup();
  rewrite_bsp::set_cluster_power(true);
  rewrite_bsp::delay_ms(prism_power_stabilize_delay_ms);
  rewrite_prism::setup();
  hardware_reset();
  eth.setSPISpeed(30000000);
  eth.config(IPAddress(192, 168, 10, rewrite_cluster_address::read()));
  ethernet_spi.begin();
  initialized = eth.begin();

  if (initialized) {
    server.begin();
    server_started = true;
  }
}

void loop()
{
  if (prism_setup_pending &&
      (millis() - prism_setup_start_ms) >= prism_power_stabilize_delay_ms) {
    rewrite_prism::setup();
    prism_setup_pending = false;
  }

  if (!server_started) {
    return;
  }

  if (!active_client || !active_client.connected()) {
    active_client = server.accept();
    if (active_client) {
      active_client.setNoDelay(true);
    }
  }

  if (!active_client || !active_client.connected()) {
    return;
  }

  const int available_byte_count = active_client.available();
  if (available_byte_count <= 0) {
    return;
  }

  size_t command_size = 0;
  while (active_client.available() > 0 && command_size < sizeof(command_buffer)) {
    const int byte_value = active_client.read();
    if (byte_value < 0) {
      break;
    }
    command_buffer[command_size++] = static_cast<uint8_t>(byte_value);
  }

  const size_t response_size = process_command(command_buffer, command_size);
  active_client.write(response_buffer, response_size);
  active_client.flush();
  active_client.stop();

  if (bootloader_reboot_pending) {
    rewrite_bsp::reboot_to_bootloader();
  }
}

Status status()
{
  if (!initialized) {
    return Status::init_failed;
  }

  return eth.linkStatus() == LinkON ? Status::link_up : Status::link_down;
}

} // namespace rewrite_network
