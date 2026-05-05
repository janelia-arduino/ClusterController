#include "rewrite_prism.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <TMC51X0.hpp>

namespace rewrite_prism
{
namespace
{

constexpr pin_size_t prism_spi_sck_pin = 10;
constexpr pin_size_t prism_spi_tx_pin = 11;
constexpr pin_size_t prism_spi_rx_pin = 12;
constexpr pin_size_t prism_spi_csn_pins[prism_count] = {14, 8, 7, 6, 5, 4, 3};
constexpr uint32_t prism_spi_clock_rate = 1000000;
constexpr size_t target_queue_capacity = 4;
constexpr uint8_t run_current_default = 75;
constexpr uint8_t start_velocity_default = 1;
constexpr uint8_t stop_velocity_default = 5;
constexpr uint8_t first_velocity_default = 10;
constexpr uint8_t max_velocity_default = 20;
constexpr uint8_t first_acceleration_default = 40;
constexpr uint8_t max_acceleration_default = 20;
constexpr uint8_t max_deceleration_default = 30;
constexpr uint8_t first_deceleration_default = 50;
constexpr uint8_t diagnostic_health_communicating = 1 << 0;
constexpr uint8_t diagnostic_health_communication_failure_latched = 1 << 1;
constexpr uint8_t diagnostic_health_reset_latched = 1 << 2;
constexpr uint8_t diagnostic_health_driver_error_latched = 1 << 3;
constexpr uint8_t diagnostic_health_charge_pump_undervoltage_latched = 1 << 4;
constexpr uint8_t diagnostic_health_recovery_attempted_latched = 1 << 5;
constexpr uint8_t diagnostic_health_recovery_failed_latched = 1 << 6;
constexpr uint8_t diagnostic_health_mirror_resync_required = 1 << 7;
constexpr uint8_t diagnostic_driver_stallguard = 1 << 0;
constexpr uint8_t diagnostic_driver_over_temperature_warning = 1 << 1;
constexpr uint8_t diagnostic_driver_over_temperature_shutdown = 1 << 2;
constexpr uint8_t diagnostic_driver_short_to_ground_a = 1 << 3;
constexpr uint8_t diagnostic_driver_short_to_ground_b = 1 << 4;
constexpr uint8_t diagnostic_driver_open_load_a = 1 << 5;
constexpr uint8_t diagnostic_driver_open_load_b = 1 << 6;
constexpr uint8_t diagnostic_driver_standstill = 1 << 7;
constexpr int16_t position_min_mm = 0;
constexpr int16_t position_max_mm = 550;
constexpr int16_t home_travel_limit_max_mm = 650;
constexpr int16_t home_expected_travel_tolerance_mm = 10;
const auto converter_parameters =
    tmc51x0::ConverterParameters()
        .withClockFrequencyMHz(16)
        .withMicrostepsPerRealPositionUnit(4881);
const auto driver_parameters_real =
    tmc51x0::DriverParameters()
        .withRunCurrent(run_current_default)
        .withHoldCurrent(0)
        .withHoldDelay(0)
        .withPwmOffset(25)
        .withPwmGradient(15)
        .withMotorDirection(tmc51x0::ForwardDirection)
        .withStandstillMode(tmc51x0::PassiveBrakingLsMode)
        .withStealthChopThreshold(250);
const auto controller_parameters_real =
    tmc51x0::ControllerParameters()
        .withRampMode(tmc51x0::PositionMode)
        .withMaxVelocity(max_velocity_default)
        .withMaxAcceleration(max_acceleration_default)
        .withStartVelocity(start_velocity_default)
        .withStopVelocity(stop_velocity_default)
        .withFirstVelocity(first_velocity_default)
        .withFirstAcceleration(first_acceleration_default)
        .withMaxDeceleration(max_deceleration_default)
        .withFirstDeceleration(first_deceleration_default);
const auto home_parameters_base_real =
    tmc51x0::HomeParameters()
        .withRunCurrent(50)
        .withHoldCurrent(20)
        .withTargetPosition(-500)
        .withVelocity(20)
        .withAcceleration(2)
        .withZeroWaitDuration(100);
const auto stall_parameters_base_real =
    tmc51x0::StallParameters()
        .withStallGuardThreshold(0)
        .withCoolStepThreshold(15);
const auto switch_parameters_running =
    tmc51x0::SwitchParameters()
        .withLeftStopEnabled(false)
        .withRightStopEnabled(false)
        .withInvertLeftPolarity(false)
        .withInvertRightPolarity(false);
const auto switch_parameters_paused =
    tmc51x0::SwitchParameters()
        .withLeftStopEnabled(true)
        .withRightStopEnabled(true)
        .withInvertLeftPolarity(true)
        .withInvertRightPolarity(true);
constexpr uint32_t home_status_poll_delay_ms = 2000;
SPIClassRP2040 &prism_spi = SPI1;
TMC51X0 prisms[prism_count];
bool initialized = false;
bool homed_state[prism_count] = {};
bool position_confident_state[prism_count] = {};
bool home_active_state[prism_count] = {};
HomeOutcome home_outcome_state[prism_count] = {};
bool paused_state[prism_count] = {};
int16_t queued_target_mm[prism_count][target_queue_capacity] = {};
uint8_t queued_target_head[prism_count] = {};
uint8_t queued_target_count[prism_count] = {};
uint32_t home_start_ms[prism_count] = {};
int32_t home_start_position_raw[prism_count] = {};
bool home_motion_observed[prism_count] = {};
bool home_position_fallback_allowed[prism_count] = {};
int32_t home_target_position_raw[prism_count] = {};
int16_t home_expected_start_position_mm[prism_count] = {};
uint16_t home_travel_limit_mm[prism_count] = {};
uint16_t home_target_travel_mm[prism_count] = {};
bool home_stallguard_fallback_state[prism_count] = {};
uint8_t last_home_travel_mm[prism_count] = {};
bool diagnostic_communication_failure_latched[prism_count] = {};
bool diagnostic_reset_latched[prism_count] = {};
bool diagnostic_driver_error_latched[prism_count] = {};
bool diagnostic_charge_pump_undervoltage_latched[prism_count] = {};
bool diagnostic_recovery_attempted_latched[prism_count] = {};
bool diagnostic_recovery_failed_latched[prism_count] = {};
uint8_t desired_run_current_percent = run_current_default;
ControllerParameters desired_controller_parameters = {
    start_velocity_default,  stop_velocity_default, first_velocity_default,
    max_velocity_default,    first_acceleration_default,
    max_acceleration_default, max_deceleration_default,
    first_deceleration_default};

void clear_target_queue(size_t prism_address);
void issue_target_position(size_t prism_address, int16_t position_mm);
void clear_diagnostic_latches(size_t prism_address);
uint16_t read_home_travel_mm(size_t prism_address);
void capture_last_home_travel(size_t prism_address);
tmc51x0::Registers::RampStat read_ramp_status(size_t prism_address);
bool home_stall_travel_plausible(size_t prism_address);
bool home_target_reached_success_allowed(size_t prism_address);
void continue_home_without_stallguard(size_t prism_address);
bool recover_if_unhealthy(size_t prism_address);

int16_t clamp_position_mm(const int16_t position_mm)
{
  if (position_mm < position_min_mm) {
    return position_min_mm;
  }
  if (position_mm > position_max_mm) {
    return position_max_mm;
  }
  return position_mm;
}

int16_t clamp_home_travel_limit_mm(const int16_t travel_limit_mm)
{
  if (travel_limit_mm < 0) {
    return 0;
  }
  if (travel_limit_mm > home_travel_limit_max_mm) {
    return home_travel_limit_max_mm;
  }
  return travel_limit_mm;
}

void clear_home_tracking(const size_t prism_address)
{
  home_start_ms[prism_address] = 0;
  home_start_position_raw[prism_address] = 0;
  home_motion_observed[prism_address] = false;
  home_position_fallback_allowed[prism_address] = false;
  home_target_position_raw[prism_address] = 0;
  home_expected_start_position_mm[prism_address] = 0;
  home_travel_limit_mm[prism_address] = 0;
  home_target_travel_mm[prism_address] = 0;
  home_stallguard_fallback_state[prism_address] = false;
}

void clear_diagnostic_latches(const size_t prism_address)
{
  diagnostic_communication_failure_latched[prism_address] = false;
  diagnostic_reset_latched[prism_address] = false;
  diagnostic_driver_error_latched[prism_address] = false;
  diagnostic_charge_pump_undervoltage_latched[prism_address] = false;
  diagnostic_recovery_attempted_latched[prism_address] = false;
  diagnostic_recovery_failed_latched[prism_address] = false;
}

uint16_t read_home_travel_mm(const size_t prism_address)
{
  TMC51X0 &prism = prisms[prism_address];
  const int32_t current_position_raw = prism.controller.readActualPosition();
  const int32_t raw_delta =
      current_position_raw >= home_start_position_raw[prism_address]
          ? current_position_raw - home_start_position_raw[prism_address]
          : home_start_position_raw[prism_address] - current_position_raw;
  const int32_t travel_mm = prism.converter.positionChipToReal(raw_delta);
  if (travel_mm <= 0) {
    return 0;
  }
  if (travel_mm > UINT16_MAX) {
    return UINT16_MAX;
  }
  return static_cast<uint16_t>(travel_mm);
}

void capture_last_home_travel(const size_t prism_address)
{
  const uint16_t travel_mm = read_home_travel_mm(prism_address);
  last_home_travel_mm[prism_address] =
      travel_mm > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(travel_mm);
}

tmc51x0::Registers::RampStat read_ramp_status(const size_t prism_address)
{
  tmc51x0::Registers::RampStat ramp_status;
  ramp_status.raw =
      prisms[prism_address].registers.read(tmc51x0::Registers::RampStatAddress);
  return ramp_status;
}

bool home_stall_travel_plausible(const size_t prism_address)
{
  if (!position_confident_state[prism_address]) {
    return false;
  }

  const uint16_t travel_mm = read_home_travel_mm(prism_address);
  const int16_t expected_start_position_mm =
      home_expected_start_position_mm[prism_address];
  if (expected_start_position_mm <= home_expected_travel_tolerance_mm) {
    return true;
  }
  return (static_cast<int32_t>(travel_mm) +
          home_expected_travel_tolerance_mm) >= expected_start_position_mm;
}

bool home_target_reached_success_allowed(const size_t prism_address)
{
  if (home_travel_limit_mm[prism_address] >= position_max_mm) {
    return true;
  }

  const int16_t expected_start_position_mm =
      home_expected_start_position_mm[prism_address];
  return position_confident_state[prism_address] &&
         expected_start_position_mm <=
             static_cast<int16_t>(home_travel_limit_mm[prism_address] +
                                  home_expected_travel_tolerance_mm);
}

void continue_home_without_stallguard(const size_t prism_address)
{
  TMC51X0 &prism = prisms[prism_address];
  home_stallguard_fallback_state[prism_address] = true;
  prism.controller.writeRampMode(tmc51x0::HoldMode);
  prism.controller.disableStallStop();
  (void)read_ramp_status(prism_address);
  prism.controller.writeTargetPosition(home_target_position_raw[prism_address]);
  prism.controller.writeRampMode(tmc51x0::PositionMode);
}

bool recover_if_unhealthy(const size_t prism_address)
{
  TMC51X0 &prism = prisms[prism_address];
  const tmc51x0::HealthStatus health_status = prism.readHealthStatus();
  const bool expected_home_stall_driver_error =
      health_status.communication_ok &&
      !health_status.reset &&
      !health_status.charge_pump_undervoltage &&
      health_status.driver_error &&
      !home_active_state[prism_address] &&
      home_outcome_state[prism_address] == HomeOutcome::stall;
  const bool driver_error =
      health_status.driver_error && !expected_home_stall_driver_error;
  const bool unhealthy = !health_status.communication_ok ||
                         health_status.reset ||
                         driver_error ||
                         health_status.charge_pump_undervoltage;

  if (!health_status.communication_ok) {
    diagnostic_communication_failure_latched[prism_address] = true;
  }
  if (health_status.reset) {
    diagnostic_reset_latched[prism_address] = true;
  }
  if (driver_error) {
    diagnostic_driver_error_latched[prism_address] = true;
  }
  if (health_status.charge_pump_undervoltage) {
    diagnostic_charge_pump_undervoltage_latched[prism_address] = true;
  }
  if (unhealthy || health_status.mirror_resync_required) {
    diagnostic_recovery_attempted_latched[prism_address] = true;
  }
  if (unhealthy) {
    prism.notePossibleMirrorDrift();
    position_confident_state[prism_address] = false;
  }

  const bool recovered = prism.recoverIfNeeded();
  if (!recovered) {
    diagnostic_recovery_failed_latched[prism_address] = true;
  }
  return recovered;
}

void restore_runtime_configuration(const size_t prism_address)
{
  TMC51X0 &prism = prisms[prism_address];
  const auto driver_parameters_real_current =
      driver_parameters_real.withRunCurrent(desired_run_current_percent);
  const auto controller_parameters_real_current =
      tmc51x0::ControllerParameters()
          .withRampMode(tmc51x0::PositionMode)
          .withMaxVelocity(desired_controller_parameters.max_velocity)
          .withMaxAcceleration(desired_controller_parameters.max_acceleration)
          .withStartVelocity(desired_controller_parameters.start_velocity)
          .withStopVelocity(desired_controller_parameters.stop_velocity)
          .withFirstVelocity(desired_controller_parameters.first_velocity)
          .withFirstAcceleration(
              desired_controller_parameters.first_acceleration)
          .withMaxDeceleration(
              desired_controller_parameters.max_deceleration)
          .withFirstDeceleration(
              desired_controller_parameters.first_deceleration);

  prism.driver.setup(
      prism.converter.driverParametersRealToChip(driver_parameters_real_current));
  prism.controller.setup(
      prism.converter.controllerParametersRealToChip(
          controller_parameters_real_current));
  prism.controller.setupSwitches(paused_state[prism_address]
                                     ? switch_parameters_paused
                                     : switch_parameters_running);
  prism.controller.writeRampMode(paused_state[prism_address]
                                     ? tmc51x0::HoldMode
                                     : tmc51x0::PositionMode);
}

void complete_home_success(const size_t prism_address, const HomeOutcome outcome)
{
  TMC51X0 &prism = prisms[prism_address];

  capture_last_home_travel(prism_address);

  // Freeze motion before zeroing so a successful home never commands a
  // corrective move away from the physical hardstop.
  prism.controller.writeRampMode(tmc51x0::HoldMode);
  prism.controller.zeroActualPosition();
  prism.controller.zeroTargetPosition();
  restore_runtime_configuration(prism_address);
  prism.controller.zeroActualPosition();
  prism.controller.zeroTargetPosition();

  homed_state[prism_address] = true;
  position_confident_state[prism_address] = true;
  home_active_state[prism_address] = false;
  home_outcome_state[prism_address] = outcome;
  clear_home_tracking(prism_address);
  clear_target_queue(prism_address);
}

void complete_home_failure(const size_t prism_address,
                           const HomeOutcome outcome = HomeOutcome::failed)
{
  TMC51X0 &prism = prisms[prism_address];
  capture_last_home_travel(prism_address);
  prism.controller.writeRampMode(tmc51x0::HoldMode);
  restore_runtime_configuration(prism_address);
  prism.controller.writeRampMode(tmc51x0::HoldMode);
  homed_state[prism_address] = false;
  if (outcome == HomeOutcome::failed) {
    position_confident_state[prism_address] = false;
  }
  home_active_state[prism_address] = false;
  home_outcome_state[prism_address] = outcome;
  clear_home_tracking(prism_address);
  clear_target_queue(prism_address);
}

void clear_target_queue(const size_t prism_address)
{
  queued_target_head[prism_address] = 0;
  queued_target_count[prism_address] = 0;
  for (size_t index = 0; index < target_queue_capacity; ++index) {
    queued_target_mm[prism_address][index] = 0;
  }
}

bool enqueue_target(const size_t prism_address, const int16_t position_mm)
{
  if (queued_target_count[prism_address] >= target_queue_capacity) {
    return false;
  }

  const uint8_t slot =
      (queued_target_head[prism_address] + queued_target_count[prism_address]) %
      target_queue_capacity;
  queued_target_mm[prism_address][slot] = clamp_position_mm(position_mm);
  ++queued_target_count[prism_address];
  return true;
}

bool dequeue_target(const size_t prism_address, int16_t &position_mm)
{
  if (queued_target_count[prism_address] == 0) {
    return false;
  }

  position_mm = queued_target_mm[prism_address][queued_target_head[prism_address]];
  queued_target_head[prism_address] =
      (queued_target_head[prism_address] + 1) % target_queue_capacity;
  --queued_target_count[prism_address];
  return true;
}

void issue_target_position(const size_t prism_address, const int16_t position_mm)
{
  TMC51X0 &prism = prisms[prism_address];
  prism.controller.writeRampMode(tmc51x0::PositionMode);
  prism.controller.writeTargetPosition(
      prism.converter.positionRealToChip(clamp_position_mm(position_mm)));
}

bool effectively_position_reached(const size_t prism_address)
{
  TMC51X0 &prism = prisms[prism_address];
  if (prism.controller.positionReached()) {
    return true;
  }
  // Some channels intermittently report a stale "position not reached" state
  // immediately after a completed home even when XACTUAL already equals XTARGET.
  return prism.controller.readActualPosition() == prism.controller.readTargetPosition();
}

void configure_defaults(const size_t prism_address)
{
  TMC51X0 &prism = prisms[prism_address];
  const auto driver_parameters_real_current =
      driver_parameters_real.withRunCurrent(desired_run_current_percent);
  const auto controller_parameters_real_current =
      tmc51x0::ControllerParameters()
          .withRampMode(tmc51x0::PositionMode)
          .withMaxVelocity(desired_controller_parameters.max_velocity)
          .withMaxAcceleration(desired_controller_parameters.max_acceleration)
          .withStartVelocity(desired_controller_parameters.start_velocity)
          .withStopVelocity(desired_controller_parameters.stop_velocity)
          .withFirstVelocity(desired_controller_parameters.first_velocity)
          .withFirstAcceleration(
              desired_controller_parameters.first_acceleration)
          .withMaxDeceleration(
              desired_controller_parameters.max_deceleration)
          .withFirstDeceleration(
              desired_controller_parameters.first_deceleration);
  prism.reinitialize();
  prism.converter.setup(converter_parameters);
  prism.driver.setup(
      prism.converter.driverParametersRealToChip(driver_parameters_real_current));
  prism.controller.setup(
      prism.converter.controllerParametersRealToChip(
          controller_parameters_real_current));
  prism.driver.enable();
  prism.controller.zeroActualPosition();
}

} // namespace

void setup()
{
  prism_spi.setSCK(prism_spi_sck_pin);
  prism_spi.setTX(prism_spi_tx_pin);
  prism_spi.setRX(prism_spi_rx_pin);
  prism_spi.begin();

  for (size_t prism_address = 0; prism_address < prism_count; ++prism_address) {
    homed_state[prism_address] = false;
    position_confident_state[prism_address] = false;
    home_active_state[prism_address] = false;
    home_outcome_state[prism_address] = HomeOutcome::none;
    paused_state[prism_address] = false;
    clear_target_queue(prism_address);
    home_start_ms[prism_address] = 0;
    home_start_position_raw[prism_address] = 0;
    home_motion_observed[prism_address] = false;
    home_position_fallback_allowed[prism_address] = false;
    home_target_position_raw[prism_address] = 0;
    home_expected_start_position_mm[prism_address] = 0;
    home_travel_limit_mm[prism_address] = 0;
    home_target_travel_mm[prism_address] = 0;
    home_stallguard_fallback_state[prism_address] = false;
    last_home_travel_mm[prism_address] = 0;
    clear_diagnostic_latches(prism_address);
    const auto spi_parameters =
        tmc51x0::SpiParameters()
            .withSpi(&prism_spi)
            .withClockRate(prism_spi_clock_rate)
            .withChipSelectPin(prism_spi_csn_pins[prism_address]);
    prisms[prism_address].setupSpi(spi_parameters,
                                   tmc51x0::Registers::DeviceModel::TMC5130A);
    prisms[prism_address].converter.setup(converter_parameters);
    if (prisms[prism_address].communicating()) {
      configure_defaults(prism_address);
    }
  }

  initialized = true;
}

void shutdown()
{
  initialized = false;
  for (size_t prism_address = 0; prism_address < prism_count; ++prism_address) {
    homed_state[prism_address] = false;
    position_confident_state[prism_address] = false;
    home_active_state[prism_address] = false;
    home_outcome_state[prism_address] = HomeOutcome::none;
    paused_state[prism_address] = false;
    clear_target_queue(prism_address);
    home_start_ms[prism_address] = 0;
    home_start_position_raw[prism_address] = 0;
    home_motion_observed[prism_address] = false;
    home_position_fallback_allowed[prism_address] = false;
    home_target_position_raw[prism_address] = 0;
    home_expected_start_position_mm[prism_address] = 0;
    home_travel_limit_mm[prism_address] = 0;
    home_target_travel_mm[prism_address] = 0;
    home_stallguard_fallback_state[prism_address] = false;
    last_home_travel_mm[prism_address] = 0;
    clear_diagnostic_latches(prism_address);
  }
}

void loop()
{
  if (!initialized) {
    return;
  }

  for (size_t prism_address = 0; prism_address < prism_count; ++prism_address) {
    if (!communicating(prism_address)) {
      diagnostic_communication_failure_latched[prism_address] = true;
      if (home_active_state[prism_address]) {
        home_outcome_state[prism_address] = HomeOutcome::failed;
      }
      homed_state[prism_address] = false;
      position_confident_state[prism_address] = false;
      home_active_state[prism_address] = false;
      continue;
    }

    TMC51X0 &prism = prisms[prism_address];

    if (!home_active_state[prism_address]) {
      if (!paused_state[prism_address] && queued_target_count[prism_address] > 0 &&
          effectively_position_reached(prism_address)) {
        int16_t next_target_mm = 0;
        if (dequeue_target(prism_address, next_target_mm)) {
          issue_target_position(prism_address, next_target_mm);
        }
      }
      (void)recover_if_unhealthy(prism_address);
      continue;
    }

    const int32_t current_position_raw = prism.controller.readActualPosition();
    if (!home_motion_observed[prism_address] &&
        current_position_raw != home_start_position_raw[prism_address]) {
      home_motion_observed[prism_address] = true;
    }

    const auto ramp_status = read_ramp_status(prism_address);
    if (!home_stallguard_fallback_state[prism_address] &&
        home_motion_observed[prism_address] &&
        ramp_status.event_stop_sg()) {
      if (home_stall_travel_plausible(prism_address)) {
        complete_home_success(prism_address, HomeOutcome::stall);
      } else if (home_travel_limit_mm[prism_address] >= position_max_mm) {
        continue_home_without_stallguard(prism_address);
      } else {
        complete_home_failure(prism_address, HomeOutcome::failed);
      }
      continue;
    }

    if (!home_motion_observed[prism_address] &&
        (millis() - home_start_ms[prism_address]) >= home_status_poll_delay_ms) {
      complete_home_failure(prism_address, HomeOutcome::failed);
      continue;
    }

    if (home_position_fallback_allowed[prism_address] &&
        ramp_status.position_reached()) {
      if (home_target_reached_success_allowed(prism_address)) {
        complete_home_success(prism_address, HomeOutcome::target_reached);
      } else {
        complete_home_failure(prism_address, HomeOutcome::target_reached);
      }
      continue;
    }

    (void)recover_if_unhealthy(prism_address);
  }
}

void begin_home(const uint8_t prism_address, const HomeParameters &parameters)
{
  if (!communicating(prism_address)) {
    return;
  }

  TMC51X0 &prism = prisms[prism_address];
  const int16_t clamped_travel_limit_mm =
      clamp_home_travel_limit_mm(parameters.travel_limit);
  const int32_t previous_position_raw = prism.controller.readActualPosition();
  const int16_t previous_position_mm = static_cast<int16_t>(
      prism.converter.positionChipToReal(previous_position_raw));
  const int16_t expected_start_position_mm =
      previous_position_mm > 0 ? previous_position_mm : 0;
  uint16_t target_travel_limit_mm = clamped_travel_limit_mm;
  if (position_confident_state[prism_address] &&
      expected_start_position_mm <=
          static_cast<int16_t>(clamped_travel_limit_mm +
                               home_expected_travel_tolerance_mm)) {
    const uint16_t bounded_fallback_travel_mm =
        static_cast<uint16_t>(expected_start_position_mm +
                              home_expected_travel_tolerance_mm);
    if (bounded_fallback_travel_mm < target_travel_limit_mm) {
      target_travel_limit_mm = bounded_fallback_travel_mm;
    }
  }
  auto home_parameters_real = home_parameters_base_real;
  home_parameters_real.run_current = parameters.run_current;
  home_parameters_real.velocity = parameters.max_velocity;

  auto stall_parameters_real = stall_parameters_base_real;
  stall_parameters_real.stall_guard_threshold = parameters.stall_threshold;
  stall_parameters_real.cool_step_threshold = parameters.max_velocity / 2;

  const auto home_parameters_chip =
      prism.converter.homeParametersRealToChip(home_parameters_real);
  const auto stall_parameters_chip =
      prism.converter.stallParametersRealToChip(stall_parameters_real);

  // Re-seed the prism into a known runtime state before homing so home does
  // not inherit stale motion configuration.
  configure_defaults(prism_address);
  prism.controller.writeActualPosition(previous_position_raw);
  prism.controller.setupSwitches(paused_state[prism_address]
                                     ? switch_parameters_paused
                                     : switch_parameters_running);
  prism.driver.writeRunCurrent(home_parameters_chip.run_current);
  prism.driver.writeHoldCurrent(home_parameters_chip.hold_current);
  prism.driver.writeHoldDelay(0);
  prism.driver.writeStallGuardThreshold(stall_parameters_chip.stall_guard_threshold);
  prism.driver.disableStallGuardFilter();
  prism.driver.disableStealthChop();
  prism.driver.disableCoolStep();
  prism.driver.writeCoolStepThreshold(stall_parameters_chip.cool_step_threshold);
  prism.driver.writeChopperMode(tmc51x0::SpreadCycleMode);
  prism.controller.writeStopMode(tmc51x0::HardMode);
  prism.controller.enableStallStop();
  prism.controller.writeRampMode(tmc51x0::HoldMode);
  prism.controller.writeMaxVelocity(home_parameters_chip.velocity);
  prism.controller.writeMaxAcceleration(home_parameters_chip.acceleration);
  prism.controller.writeZeroWaitDuration(home_parameters_chip.zero_wait_duration);
  home_target_position_raw[prism_address] =
      previous_position_raw -
      prism.converter.positionRealToChip(target_travel_limit_mm);
  prism.controller.writeTargetPosition(home_target_position_raw[prism_address]);
  homed_state[prism_address] = false;
  home_outcome_state[prism_address] = HomeOutcome::in_progress;
  home_start_position_raw[prism_address] = prism.controller.readActualPosition();
  home_motion_observed[prism_address] = false;
  home_position_fallback_allowed[prism_address] = true;
  home_expected_start_position_mm[prism_address] = expected_start_position_mm;
  home_travel_limit_mm[prism_address] = clamped_travel_limit_mm;
  home_target_travel_mm[prism_address] = target_travel_limit_mm;
  home_stallguard_fallback_state[prism_address] = false;
  last_home_travel_mm[prism_address] = 0;
  clear_target_queue(prism_address);
  (void)read_ramp_status(prism_address);
  prism.controller.writeRampMode(tmc51x0::PositionMode);
  home_start_ms[prism_address] = millis();
  home_active_state[prism_address] = true;
}

bool communicating(const uint8_t prism_address)
{
  if (!initialized || prism_address >= prism_count) {
    return false;
  }

  const bool ok = prisms[prism_address].communicating();
  if (!ok) {
    diagnostic_communication_failure_latched[prism_address] = true;
  }
  return ok;
}

bool homed(const uint8_t prism_address)
{
  if (!initialized || prism_address >= prism_count) {
    return false;
  }

  return homed_state[prism_address];
}

bool home_active(const uint8_t prism_address)
{
  if (!initialized || prism_address >= prism_count) {
    return false;
  }

  return home_active_state[prism_address];
}

bool home_failed(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return false;
  }

  return prisms[prism_address].homeFailed();
}

uint8_t home_outcome(const uint8_t prism_address)
{
  if (!initialized || prism_address >= prism_count) {
    return static_cast<uint8_t>(HomeOutcome::none);
  }

  return static_cast<uint8_t>(home_outcome_state[prism_address]);
}

bool paused(const uint8_t prism_address)
{
  if (!initialized || prism_address >= prism_count) {
    return false;
  }

  return paused_state[prism_address];
}

void write_target(const uint8_t prism_address, const int16_t position_mm)
{
  if (!initialized || prism_address >= prism_count) {
    return;
  }

  const int16_t clamped_position_mm = clamp_position_mm(position_mm);
  if (!communicating(prism_address)) {
    // After a successful home, a transient readVersion() miss should not cause
    // a cluster target write to disappear permanently.
    if (homed_state[prism_address]) {
      (void)enqueue_target(prism_address, clamped_position_mm);
    }
    return;
  }

  TMC51X0 &prism = prisms[prism_address];
  if (paused_state[prism_address] || !effectively_position_reached(prism_address)) {
    (void)enqueue_target(prism_address, clamped_position_mm);
    return;
  }

  clear_target_queue(prism_address);
  issue_target_position(prism_address, clamped_position_mm);
}

void pause(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return;
  }

  paused_state[prism_address] = true;
  // Switch configuration alone was not sufficient to hold position reliably on
  // the bench; force the controller into HoldMode during the pause window.
  prisms[prism_address].controller.writeRampMode(tmc51x0::HoldMode);
  prisms[prism_address].controller.setupSwitches(switch_parameters_paused);
}

void resume(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return;
  }

  paused_state[prism_address] = false;
  prisms[prism_address].controller.setupSwitches(switch_parameters_running);
  prisms[prism_address].controller.writeRampMode(tmc51x0::PositionMode);
  if (queued_target_count[prism_address] > 0 &&
      effectively_position_reached(prism_address)) {
    int16_t next_target_mm = 0;
    if (dequeue_target(prism_address, next_target_mm)) {
      issue_target_position(prism_address, next_target_mm);
    }
  }
}

int16_t read_position_mm(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return unhomed_position;
  }

  return static_cast<int16_t>(
      prisms[prism_address].converter.positionChipToReal(
          prisms[prism_address].controller.readActualPosition()));
}

void write_run_current(const uint8_t prism_address,
                       const uint8_t run_current_percent)
{
  desired_run_current_percent = run_current_percent;

  if (!communicating(prism_address)) {
    return;
  }

  prisms[prism_address].driver.writeRunCurrent(
      prisms[prism_address].converter.percentToCurrentSetting(
          run_current_percent));
}

void write_controller_parameters(const uint8_t prism_address,
                                 const ControllerParameters &parameters)
{
  desired_controller_parameters = parameters;

  if (!communicating(prism_address)) {
    return;
  }

  TMC51X0 &prism = prisms[prism_address];
  prism.controller.writeStartVelocity(
      prism.converter.velocityRealToChip(parameters.start_velocity));
  prism.controller.writeStopVelocity(
      prism.converter.velocityRealToChip(parameters.stop_velocity));
  prism.controller.writeFirstVelocity(
      prism.converter.velocityRealToChip(parameters.first_velocity));
  prism.controller.writeMaxVelocity(
      prism.converter.velocityRealToChip(parameters.max_velocity));
  prism.controller.writeFirstAcceleration(
      prism.converter.accelerationRealToChip(parameters.first_acceleration));
  prism.controller.writeMaxAcceleration(
      prism.converter.accelerationRealToChip(parameters.max_acceleration));
  prism.controller.writeMaxDeceleration(
      prism.converter.accelerationRealToChip(parameters.max_deceleration));
  prism.controller.writeFirstDeceleration(
      prism.converter.accelerationRealToChip(parameters.first_deceleration));
}

PrismDiagnostics read_diagnostics(const uint8_t prism_address)
{
  PrismDiagnostics diagnostics{};
  if (!initialized || prism_address >= prism_count) {
    return diagnostics;
  }

  const bool communication_ok = communicating(prism_address);
  if (communication_ok) {
    diagnostics.health_flags |= diagnostic_health_communicating;
  }
  if (diagnostic_communication_failure_latched[prism_address]) {
    diagnostics.health_flags |= diagnostic_health_communication_failure_latched;
  }
  if (diagnostic_reset_latched[prism_address]) {
    diagnostics.health_flags |= diagnostic_health_reset_latched;
  }
  if (diagnostic_driver_error_latched[prism_address]) {
    diagnostics.health_flags |= diagnostic_health_driver_error_latched;
  }
  if (diagnostic_charge_pump_undervoltage_latched[prism_address]) {
    diagnostics.health_flags |= diagnostic_health_charge_pump_undervoltage_latched;
  }
  if (diagnostic_recovery_attempted_latched[prism_address]) {
    diagnostics.health_flags |= diagnostic_health_recovery_attempted_latched;
  }
  if (diagnostic_recovery_failed_latched[prism_address]) {
    diagnostics.health_flags |= diagnostic_health_recovery_failed_latched;
  }
  if (communication_ok && prisms[prism_address].mirrorResyncRequired()) {
    diagnostics.health_flags |= diagnostic_health_mirror_resync_required;
  }

  if (!communication_ok) {
    return diagnostics;
  }

  tmc51x0::Registers::DrvStatus driver_status;
  driver_status.raw =
      prisms[prism_address].registers.read(tmc51x0::Registers::DrvStatusAddress);
  if (driver_status.stallguard()) {
    diagnostics.driver_flags |= diagnostic_driver_stallguard;
  }
  if (driver_status.otpw()) {
    diagnostics.driver_flags |= diagnostic_driver_over_temperature_warning;
  }
  if (driver_status.ot()) {
    diagnostics.driver_flags |= diagnostic_driver_over_temperature_shutdown;
  }
  if (driver_status.s2ga()) {
    diagnostics.driver_flags |= diagnostic_driver_short_to_ground_a;
  }
  if (driver_status.s2gb()) {
    diagnostics.driver_flags |= diagnostic_driver_short_to_ground_b;
  }
  if (driver_status.ola()) {
    diagnostics.driver_flags |= diagnostic_driver_open_load_a;
  }
  if (driver_status.olb()) {
    diagnostics.driver_flags |= diagnostic_driver_open_load_b;
  }
  if (driver_status.stst()) {
    diagnostics.driver_flags |= diagnostic_driver_standstill;
  }
  diagnostics.stall_guard_result =
      static_cast<uint16_t>(driver_status.sg_result());
  diagnostics.current_scale = driver_status.cs_actual();
  diagnostics.last_home_travel_mm = last_home_travel_mm[prism_address];
  return diagnostics;
}

void clear_diagnostics(const uint8_t prism_address)
{
  if (!initialized || prism_address >= prism_count) {
    return;
  }
  clear_diagnostic_latches(prism_address);
}

} // namespace rewrite_prism
