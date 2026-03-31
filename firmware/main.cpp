#if defined(CLUSTER_CONTROLLER_REWRITE)
#include "rewrite_app.hpp"
#else
#include "ClusterController.hpp"
#endif

#include <Arduino.h>

extern "C" __attribute__((used)) void setup()
{
#if defined(CLUSTER_CONTROLLER_REWRITE)
  rewrite_app::setup();
#else
  ArduinoInterface::setup();
#endif
}

extern "C" __attribute__((used)) void loop()
{
#if defined(CLUSTER_CONTROLLER_REWRITE)
  rewrite_app::loop();
#else
  ArduinoInterface::loop();
#endif
}
