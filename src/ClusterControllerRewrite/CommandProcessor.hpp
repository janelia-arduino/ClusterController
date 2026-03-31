#pragma once

#include <stdint.h>

namespace cluster_controller_rewrite
{

class CommandProcessor
{
public:
  void setup();
  void tick();
  uint8_t protocolVersion() const;
};

} // namespace cluster_controller_rewrite
