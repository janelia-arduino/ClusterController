#pragma once

#include <stdint.h>

namespace cluster_controller_rewrite
{

class Bsp
{
public:
  void setup();
  void tick();

private:
  uint32_t next_heartbeat_ms_ = 0;
  bool led_on_ = false;

  void printBootBanner() const;
};

} // namespace cluster_controller_rewrite
