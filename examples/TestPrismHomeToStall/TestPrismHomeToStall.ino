#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI1;
size_t SCK_PIN = 10;
size_t TX_PIN = 11;
size_t RX_PIN = 12;
#else
SPIClass & spi = SPI;
#endif

const tmc51x0::SpiParameters spi_parameters =
{
  .spi_ptr = &spi,
  .chip_select_pin = 14
};

const tmc51x0::ConverterParameters converter_parameters =
{
  .clock_frequency_mhz = 16,
  .microsteps_per_real_position_unit = 4881
};
// external clock is 16MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49 ~= 4881
// one "real unit" in this example is one millimeters of linear travel

const tmc51x0::DriverParameters driver_parameters_real =
{
  .global_current_scaler = 50, // (percent)
  .run_current = 50, // (percent)
  .hold_current = 20, // (percent)
  .hold_delay = 0, // (percent)
  .pwm_offset = 15, // (percent)
  .pwm_gradient = 5, // (percent)
  .motor_direction = tmc51x0::ReverseDirection,
  .stealth_chop_threshold = 10, // (millimeters/s)
  .cool_step_threshold = 50, // (millimeters/s)
  .cool_step_enabled = true,
  .stall_guard_threshold = 1,
};

const tmc51x0::ControllerParameters controller_parameters_real =
{
  .ramp_mode = tmc51x0::PositionMode,
  .max_velocity = 20, // (millimeters/s)
  .max_acceleration = 2, // ((millimeters/s)/s)
  .start_velocity = 1, // (millimeters/s)
  .stop_velocity = 5, // (millimeters/s)
  .first_velocity = 10, // (millimeters/s)
  .first_acceleration = 10, // ((millimeters/s)/s)
  .max_deceleration = 20, // ((millimeters/s)/s)
  .first_deceleration = 25, // ((millimeters/s)/s)
};

const tmc51x0::HomeParameters home_parameters_real =
{
  .run_current = 50, // (percent)
  .hold_current = 20, // (percent)
  .target_position = -1000, // (millimeters)
  .velocity = 20, // (millimeters/s)
  .acceleration = 2, // ((millimeters/s)/s)
  .zero_wait_duration = 100 // (milliseconds)
};

const tmc51x0::StallParameters stall_parameters_real =
{
  .stall_guard_threshold = 10,
  .cool_step_threshold = 15 // (millimeters/s)
};

const int32_t MOVE_POSITION = 100;  // millimeters

const size_t ENABLE_POWER_PIN = 15;
const uint8_t ENABLE_POWER_POLARITY = HIGH;
const uint16_t RESET_DELAY = 5000;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t PAUSE_DELAY = 4000;

// global variables
TMC51X0 prism;
tmc51x0::ControllerParameters controller_parameters_chip;
tmc51x0::HomeParameters home_parameters_chip;
tmc51x0::StallParameters stall_parameters_chip;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  prism.setEnablePowerPin(ENABLE_POWER_PIN);
  prism.setEnablePowerPolarity(ENABLE_POWER_POLARITY);
  prism.disablePower();
  delay(RESET_DELAY);
  prism.enablePower();
  delay(RESET_DELAY);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  prism.setupSpi(spi_parameters);

  prism.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = prism.converter.driverParametersRealToChip(driver_parameters_real);
  prism.driver.setup(driver_parameters_chip);

  controller_parameters_chip = prism.converter.controllerParametersRealToChip(controller_parameters_real);
  prism.controller.setup(controller_parameters_chip);

  home_parameters_chip = prism.converter.homeParametersRealToChip(home_parameters_real);
  stall_parameters_chip = prism.converter.stallParametersRealToChip(stall_parameters_real);

  while (!prism.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(LOOP_DELAY);
  }

  while (prism.controller.stepAndDirectionMode())
  {
    Serial.println("Step and Direction mode enabled so SPI/UART motion commands will not work!");
    delay(LOOP_DELAY);
  }

  prism.driver.enable();

  prism.controller.beginRampToZeroVelocity();
  while (not prism.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  prism.controller.endRampToZeroVelocity();
}

void loop()
{
  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  Serial.println("Homing to stall...");
  prism.beginHomeToStall(home_parameters_chip, stall_parameters_chip);

  int32_t actual_position_real;
  while (not prism.homed())
  {
    // prism.printer.readAndPrintDrvStatus();
    int32_t actual_position_chip = prism.controller.readActualPosition();
    actual_position_real = prism.converter.positionChipToReal(actual_position_chip);
    Serial.print("homing...");
    Serial.print("actual position (millimeters): ");
    Serial.println(actual_position_real);
    Serial.print("stall guard result: ");
    Serial.println(prism.driver.readStallGuardResult());
    Serial.print("stall guard threshold: ");
    Serial.println(stall_parameters_real.stall_guard_threshold);
    delay(LOOP_DELAY);
  }
  prism.endHome();
  Serial.println("Homed!");
  Serial.print("actual_position_real: ");
  Serial.println(actual_position_real);
  Serial.print("home target position: ");
  Serial.println(home_parameters_real.target_position);

  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  int32_t target_position_chip = prism.converter.positionRealToChip(MOVE_POSITION);
  prism.controller.writeTargetPosition(target_position_chip);
  Serial.print("Moving to another position (millimeters): ");
  Serial.print(MOVE_POSITION);
  Serial.println("...");

  while (not prism.controller.positionReached())
  {
    // prism.printer.readAndPrintRampStat();
    // prism.printer.readAndPrintDrvStatus();
    int32_t actual_position_chip = prism.controller.readActualPosition();
    int32_t actual_position_real = prism.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (millimeters): ");
    Serial.println(actual_position_real);
    Serial.print("stall_guard_result: ");
    Serial.println(prism.driver.readStallGuardResult());
    delay(LOOP_DELAY);
  }
  Serial.println("Target position reached!");
  delay(PAUSE_DELAY);

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
