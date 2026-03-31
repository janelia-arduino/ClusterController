#include "rewrite_app.hpp"

#include "rewrite_bsp.hpp"
#include "rewrite_network.hpp"
#include "rewrite_prism.hpp"

namespace rewrite_app
{
namespace
{

bool led_state = false;

} // namespace

void setup()
{
  rewrite_bsp::setup();
  rewrite_network::setup();
}

void loop()
{
  rewrite_prism::loop();
  rewrite_network::loop();

  led_state = !led_state;
  switch (rewrite_network::status()) {
    case rewrite_network::Status::init_failed:
      rewrite_bsp::set_status_led(led_state);
      rewrite_bsp::delay_ms(1000);
      break;

    case rewrite_network::Status::link_down:
      rewrite_bsp::set_status_led(led_state);
      rewrite_bsp::delay_ms(500);
      break;

    case rewrite_network::Status::link_up:
      rewrite_bsp::set_status_led(led_state);
      rewrite_bsp::delay_ms(150);
      break;
  }
}

} // namespace rewrite_app
