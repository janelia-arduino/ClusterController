#include "rewrite_app.hpp"

#include "rewrite_bsp.hpp"
#include "rewrite_network.hpp"
#include "rewrite_prism.hpp"

#include <Arduino.h>

namespace rewrite_app
{
void setup()
{
  rewrite_bsp::setup();
  rewrite_network::setup();
  rewrite_bsp::set_status_led(false);
}

void loop()
{
  rewrite_prism::loop();
  rewrite_network::loop();
}

} // namespace rewrite_app
