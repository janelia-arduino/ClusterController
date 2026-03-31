#pragma once

namespace rewrite_network
{

enum class Status
{
  init_failed,
  link_down,
  link_up,
};

void setup();
void loop();
Status status();

} // namespace rewrite_network
