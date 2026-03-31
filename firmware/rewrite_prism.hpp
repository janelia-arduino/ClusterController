#pragma once

#include <stdint.h>
#include <stddef.h>

namespace rewrite_prism
{

constexpr size_t prism_count = 7;
constexpr int16_t unhomed_position = -1;

struct ControllerParameters {
  uint8_t start_velocity;
  uint8_t stop_velocity;
  uint8_t first_velocity;
  uint8_t max_velocity;
  uint8_t first_acceleration;
  uint8_t max_acceleration;
  uint8_t max_deceleration;
  uint8_t first_deceleration;
};

struct HomeParameters {
  uint16_t travel_limit;
  uint8_t max_velocity;
  uint8_t run_current;
  int8_t stall_threshold;
};

void setup();
void shutdown();
void loop();
void begin_home(uint8_t prism_address, const HomeParameters &parameters);
bool communicating(uint8_t prism_address);
bool homed(uint8_t prism_address);
bool home_active(uint8_t prism_address);
bool home_failed(uint8_t prism_address);
bool step_and_direction_mode(uint8_t prism_address);
int32_t read_position_raw(uint8_t prism_address);
int32_t read_velocity_raw(uint8_t prism_address);
uint32_t read_max_velocity_raw(uint8_t prism_address);
uint32_t read_max_acceleration_raw(uint8_t prism_address);
void pause(uint8_t prism_address);
void resume(uint8_t prism_address);
int16_t read_position_mm(uint8_t prism_address);
void write_run_current(uint8_t prism_address, uint8_t run_current_percent);
void write_controller_parameters(uint8_t prism_address,
                                 const ControllerParameters &parameters);

} // namespace rewrite_prism
