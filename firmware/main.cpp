#include "rewrite_app.hpp"

#include <Arduino.h>

extern "C" __attribute__((used)) void setup()
{
  rewrite_app::setup();
}

extern "C" __attribute__((used)) void loop()
{
  rewrite_app::loop();
}
