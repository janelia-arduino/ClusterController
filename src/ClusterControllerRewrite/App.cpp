#include "ClusterControllerRewrite/App.hpp"

namespace cluster_controller_rewrite
{

void App::setup()
{
  bsp_.setup();
  commands_.setup();
  network_.setup(commands_.protocolVersion());
}

void App::loop()
{
  bsp_.tick();
  network_.tick();
  commands_.tick();
}

} // namespace cluster_controller_rewrite
