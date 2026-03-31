#include "rewrite_prism.hpp"

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
constexpr uint8_t run_current_default = 75;
constexpr uint8_t start_velocity_default = 1;
constexpr uint8_t stop_velocity_default = 5;
constexpr uint8_t first_velocity_default = 10;
constexpr uint8_t max_velocity_default = 20;
constexpr uint8_t first_acceleration_default = 40;
constexpr uint8_t max_acceleration_default = 20;
constexpr uint8_t max_deceleration_default = 30;
constexpr uint8_t first_deceleration_default = 50;
constexpr uint32_t zero_velocity_poll_delay_ms = 50;
constexpr uint32_t zero_velocity_timeout_ms = 3000;
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

SPIClassRP2040 &prism_spi = SPI1;
TMC51X0 prisms[prism_count];
bool initialized = false;
bool homed_state[prism_count] = {};
bool home_active_state[prism_count] = {};

void configure_defaults(const size_t prism_address)
{
  TMC51X0 &prism = prisms[prism_address];
  prism.reinitialize();
  prism.converter.setup(converter_parameters);
  prism.driver.setup(
      prism.converter.driverParametersRealToChip(driver_parameters_real));
  prism.controller.setup(
      prism.converter.controllerParametersRealToChip(controller_parameters_real));
  prism.controller.setupSwitches(switch_parameters_running);
  prism.driver.enable();
  prism.controller.beginRampToZeroVelocity();
  const uint32_t start_ms = millis();
  while (!prism.controller.zeroVelocity()) {
    if ((millis() - start_ms) >= zero_velocity_timeout_ms) {
      break;
    }
    delay(zero_velocity_poll_delay_ms);
  }
  prism.controller.endRampToZeroVelocity();
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
  }
}

void loop()
{
  if (!initialized) {
    return;
  }

  for (size_t prism_address = 0; prism_address < prism_count; ++prism_address) {
    if (!communicating(prism_address)) {
      homed_state[prism_address] = false;
      home_active_state[prism_address] = false;
      continue;
    }

    TMC51X0 &prism = prisms[prism_address];

    if (!home_active_state[prism_address]) {
      (void)prism.recoverIfUnhealthy();
      continue;
    }

    if (prism.homed()) {
      prism.endHome();
      homed_state[prism_address] = true;
      home_active_state[prism_address] = false;
      continue;
    }

    if (prism.homeFailed()) {
      prism.endHome();
      homed_state[prism_address] = false;
      home_active_state[prism_address] = false;
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

  tmc51x0::HomeParameters home_parameters_real = home_parameters_base_real;
  home_parameters_real.run_current = parameters.run_current;
  home_parameters_real.target_position = -1 * parameters.travel_limit;
  home_parameters_real.velocity = parameters.max_velocity;

  tmc51x0::StallParameters stall_parameters_real = stall_parameters_base_real;
  stall_parameters_real.stall_guard_threshold = parameters.stall_threshold;
  stall_parameters_real.cool_step_threshold = parameters.max_velocity / 2;

  const auto home_parameters_chip =
      prism.converter.homeParametersRealToChip(home_parameters_real);
  const auto stall_parameters_chip =
      prism.converter.stallParametersRealToChip(stall_parameters_real);

  prism.beginHomeToStall(home_parameters_chip, stall_parameters_chip);
  homed_state[prism_address] = false;
  if (prism.homed()) {
    prism.endHome();
    homed_state[prism_address] = true;
    home_active_state[prism_address] = false;
    return;
  }
  if (prism.homeFailed()) {
    prism.endHome();
    homed_state[prism_address] = false;
    home_active_state[prism_address] = false;
    return;
  }
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

bool step_and_direction_mode(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return false;
  }

  return prisms[prism_address].controller.stepAndDirectionMode();
}

int32_t read_position_raw(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return 0;
  }

  return prisms[prism_address].controller.readActualPosition();
}

int32_t read_velocity_raw(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return 0;
  }

  return prisms[prism_address].controller.readActualVelocity();
}

uint32_t read_max_velocity_raw(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return 0;
  }

  return prisms[prism_address].registers.read(tmc51x0::Registers::VmaxAddress);
}

uint32_t read_max_acceleration_raw(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return 0;
  }

  return prisms[prism_address].registers.read(tmc51x0::Registers::AmaxAddress);
}

void pause(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return;
  }

  prisms[prism_address].controller.setupSwitches(switch_parameters_paused);
}

void resume(const uint8_t prism_address)
{
  if (!communicating(prism_address)) {
    return;
  }

  prisms[prism_address].controller.setupSwitches(switch_parameters_running);
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
