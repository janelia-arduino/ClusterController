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
        .withStallGuardThreshold(10)
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
uint8_t desired_run_current_percent = run_current_default;
ControllerParameters desired_controller_parameters = {
    start_velocity_default,  stop_velocity_default, first_velocity_default,
    max_velocity_default,    first_acceleration_default,
    max_acceleration_default, max_deceleration_default,
    first_deceleration_default};

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
  queued_target_mm[prism_address][slot] = position_mm;
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
    home_active_state[prism_address] = false;
    home_outcome_state[prism_address] = HomeOutcome::none;
    paused_state[prism_address] = false;
    clear_target_queue(prism_address);
    home_start_ms[prism_address] = 0;
    home_start_position_raw[prism_address] = 0;
    home_motion_observed[prism_address] = false;
    home_position_fallback_allowed[prism_address] = false;
    home_target_position_raw[prism_address] = 0;
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
    home_active_state[prism_address] = false;
    home_outcome_state[prism_address] = HomeOutcome::none;
    paused_state[prism_address] = false;
    clear_target_queue(prism_address);
    home_start_ms[prism_address] = 0;
    home_start_position_raw[prism_address] = 0;
    home_motion_observed[prism_address] = false;
    home_position_fallback_allowed[prism_address] = false;
    home_target_position_raw[prism_address] = 0;
  }
}

void loop()
{
  if (!initialized) {
    return;
  }

  for (size_t prism_address = 0; prism_address < prism_count; ++prism_address) {
    if (!communicating(prism_address)) {
      if (home_active_state[prism_address]) {
        home_outcome_state[prism_address] = HomeOutcome::failed;
      }
      homed_state[prism_address] = false;
      home_active_state[prism_address] = false;
      continue;
    }

    TMC51X0 &prism = prisms[prism_address];

    if (!home_active_state[prism_address]) {
      if (!paused_state[prism_address] && queued_target_count[prism_address] > 0 &&
          prism.controller.positionReached()) {
        int16_t next_target_mm = 0;
        if (dequeue_target(prism_address, next_target_mm)) {
          prism.controller.writeTargetPosition(
              prism.converter.positionRealToChip(next_target_mm));
        }
      }
      (void)prism.recoverIfUnhealthy();
      continue;
    }

    const int32_t current_position_raw = prism.controller.readActualPosition();
    if (!home_motion_observed[prism_address] &&
        current_position_raw != home_start_position_raw[prism_address]) {
      home_motion_observed[prism_address] = true;
    }

    if (!home_motion_observed[prism_address] &&
        (millis() - home_start_ms[prism_address]) < home_status_poll_delay_ms) {
      (void)prism.recoverIfUnhealthy();
      continue;
    }

    if (prism.driver.stalled()) {
      prism.controller.writeTargetPosition(0);
      prism.controller.zeroActualPosition();
      prism.controller.zeroActualPosition();
      homed_state[prism_address] = true;
      home_active_state[prism_address] = false;
      home_outcome_state[prism_address] = HomeOutcome::stall;
      home_start_ms[prism_address] = 0;
      home_start_position_raw[prism_address] = 0;
      home_motion_observed[prism_address] = false;
      home_position_fallback_allowed[prism_address] = false;
      home_target_position_raw[prism_address] = 0;
      clear_target_queue(prism_address);
      continue;
    }

    if (home_position_fallback_allowed[prism_address] &&
        prism.controller.positionReached()) {
      prism.controller.writeTargetPosition(0);
      prism.controller.zeroActualPosition();
      homed_state[prism_address] = true;
      home_active_state[prism_address] = false;
      home_outcome_state[prism_address] = HomeOutcome::target_reached;
      home_start_ms[prism_address] = 0;
      home_start_position_raw[prism_address] = 0;
      home_motion_observed[prism_address] = false;
      home_position_fallback_allowed[prism_address] = false;
      home_target_position_raw[prism_address] = 0;
      clear_target_queue(prism_address);
      continue;
    }

    if (!communicating(prism_address)) {
      homed_state[prism_address] = false;
      home_active_state[prism_address] = false;
      home_outcome_state[prism_address] = HomeOutcome::failed;
      home_start_ms[prism_address] = 0;
      home_start_position_raw[prism_address] = 0;
      home_motion_observed[prism_address] = false;
      home_position_fallback_allowed[prism_address] = false;
      home_target_position_raw[prism_address] = 0;
      clear_target_queue(prism_address);
      continue;
    }

    (void)prism.recoverIfUnhealthy();
  }
}

void begin_home(const uint8_t prism_address, const HomeParameters &parameters)
{
  if (!communicating(prism_address)) {
    return;
  }

  TMC51X0 &prism = prisms[prism_address];
  auto home_parameters_real = home_parameters_base_real;
  home_parameters_real.run_current = parameters.run_current;
  home_parameters_real.target_position = -1 * static_cast<int32_t>(parameters.travel_limit);
  home_parameters_real.velocity = parameters.max_velocity;

  auto stall_parameters_real = stall_parameters_base_real;
  stall_parameters_real.stall_guard_threshold = parameters.stall_threshold;
  stall_parameters_real.cool_step_threshold = parameters.max_velocity / 2;

  const auto home_parameters_chip =
      prism.converter.homeParametersRealToChip(home_parameters_real);
  const auto stall_parameters_chip =
      prism.converter.stallParametersRealToChip(stall_parameters_real);

  // Re-seed the prism into a known controller state before homing so home
  // does not inherit an in-flight target or stale motion configuration.
  configure_defaults(prism_address);
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
  prism.controller.writeRampMode(tmc51x0::HoldMode);
  prism.controller.writeMaxVelocity(home_parameters_chip.velocity);
  prism.controller.writeMaxAcceleration(home_parameters_chip.acceleration);
  prism.controller.writeZeroWaitDuration(home_parameters_chip.zero_wait_duration);
  home_target_position_raw[prism_address] = home_parameters_chip.target_position;
  prism.controller.writeTargetPosition(home_target_position_raw[prism_address]);
  prism.controller.writeRampMode(tmc51x0::PositionMode);
  homed_state[prism_address] = false;
  home_outcome_state[prism_address] = HomeOutcome::in_progress;
  home_start_ms[prism_address] = millis();
  home_start_position_raw[prism_address] = prism.controller.readActualPosition();
  home_motion_observed[prism_address] = false;
  home_position_fallback_allowed[prism_address] = true;
  clear_target_queue(prism_address);
  home_active_state[prism_address] = true;
}

bool communicating(const uint8_t prism_address)
{
  if (!initialized || prism_address >= prism_count) {
    return false;
  }

  return prisms[prism_address].communicating();
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
  if (!communicating(prism_address)) {
    return;
  }

  TMC51X0 &prism = prisms[prism_address];
  if (paused_state[prism_address] || !prism.controller.positionReached()) {
    (void)enqueue_target(prism_address, position_mm);
    return;
  }

  clear_target_queue(prism_address);
  prism.controller.writeTargetPosition(
      prism.converter.positionRealToChip(position_mm));
}

void pause(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return;
  }

  paused_state[prism_address] = true;
  prisms[prism_address].controller.setupSwitches(switch_parameters_paused);
}

void resume(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return;
  }

  paused_state[prism_address] = false;
  prisms[prism_address].controller.setupSwitches(switch_parameters_running);
  if (queued_target_count[prism_address] > 0 &&
      prisms[prism_address].controller.positionReached()) {
    int16_t next_target_mm = 0;
    if (dequeue_target(prism_address, next_target_mm)) {
      prisms[prism_address].controller.writeTargetPosition(
          prisms[prism_address].converter.positionRealToChip(
              next_target_mm));
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

} // namespace rewrite_prism
