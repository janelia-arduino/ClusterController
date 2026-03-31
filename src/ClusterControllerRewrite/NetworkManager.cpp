#include "ClusterControllerRewrite/NetworkManager.hpp"

namespace cluster_controller_rewrite
{

void NetworkManager::setup(uint8_t protocol_version)
{
  protocol_version_ = protocol_version;
}

void NetworkManager::tick()
{
  (void)protocol_version_;
}

} // namespace cluster_controller_rewrite
