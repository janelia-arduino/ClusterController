#pragma once

#include <stdint.h>
#include <stddef.h>

namespace rewrite_prism
{

constexpr size_t prism_count = 7;
constexpr int16_t unhomed_position = -1;

enum class HomeOutcome : uint8_t {
  none = 0,
  in_progress = 1,
  stall = 2,
  target_reached = 3,
  failed = 4,
};

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
uint8_t home_outcome(uint8_t prism_address);
bool paused(uint8_t prism_address);
void write_target(uint8_t prism_address, int16_t position_mm);
void pause(uint8_t prism_address);
void resume(uint8_t prism_address);
int16_t read_position_mm(uint8_t prism_address);
void write_run_current(uint8_t prism_address, uint8_t run_current_percent);
void write_controller_parameters(uint8_t prism_address,
                                 const ControllerParameters &parameters);

} // namespace rewrite_prism
