#include "rewrite_app.hpp"

#include "rewrite_bsp.hpp"
#include "rewrite_network.hpp"
#include "rewrite_prism.hpp"

#include <Arduino.h>

namespace rewrite_app
{
namespace
{

bool led_state = false;
uint32_t last_status_led_toggle_ms = 0;

uint32_t status_led_interval_ms()
{
  switch (rewrite_network::status()) {
    case rewrite_network::Status::init_failed:
      return 1000;
    case rewrite_network::Status::link_down:
      return 500;
    case rewrite_network::Status::link_up:
      return 150;
  }

  return 1000;
}

} // namespace

void setup()
{
  rewrite_bsp::setup();
  rewrite_network::setup();
  last_status_led_toggle_ms = millis();
}

void loop()
{
  rewrite_prism::loop();
  rewrite_network::loop();

  const uint32_t now_ms = millis();
  if ((now_ms - last_status_led_toggle_ms) >= status_led_interval_ms()) {
    led_state = !led_state;
    rewrite_bsp::set_status_led(led_state);
    last_status_led_toggle_ms = now_ms;
  }
}

} // namespace rewrite_app
