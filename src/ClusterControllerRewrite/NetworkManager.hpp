#pragma once

#include <stdint.h>

namespace cluster_controller_rewrite
{

class NetworkManager
{
public:
  void setup(uint8_t protocol_version);
  void tick();

private:
  uint8_t protocol_version_ = 0;
};

} // namespace cluster_controller_rewrite
