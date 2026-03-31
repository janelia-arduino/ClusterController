#pragma once

#include "ClusterControllerRewrite/Bsp.hpp"
#include "ClusterControllerRewrite/CommandProcessor.hpp"
#include "ClusterControllerRewrite/NetworkManager.hpp"

namespace cluster_controller_rewrite
{

class App
{
public:
  void setup();
  void loop();

private:
  Bsp bsp_;
  NetworkManager network_;
  CommandProcessor commands_;
};

} // namespace cluster_controller_rewrite
