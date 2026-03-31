#include "rewrite_cluster_address.hpp"

#include <TCA6408.h>
#include <Wire.h>

namespace rewrite_cluster_address
{
namespace
{

constexpr pin_size_t sda_pin = 26;
constexpr pin_size_t scl_pin = 27;
constexpr pin_size_t cluster_address_reset_pin = 0;
constexpr TCA6408::DeviceAddress cluster_address_device_address = TCA6408::DEVICE_ADDRESS_0;
constexpr uint8_t fallback_cluster_address = 10;

TwoWire &wire = Wire1;
TCA6408 tca6408;
bool initialized = false;

} // namespace

void setup()
{
  wire.setSDA(sda_pin);
  wire.setSCL(scl_pin);
  wire.begin();

  tca6408.setup(wire, cluster_address_device_address);
  tca6408.setResetPin(cluster_address_reset_pin);
  initialized = true;
}

uint8_t read()
{
  if (!initialized) {
    return fallback_cluster_address;
  }

  return tca6408.readInputRegister();
}

} // namespace rewrite_cluster_address
