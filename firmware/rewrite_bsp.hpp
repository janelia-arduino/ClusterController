#pragma once

#include <stdint.h>

namespace rewrite_bsp
{

void setup();
void set_status_led(bool enabled);
void set_command_led(bool enabled);
void play_status_tone(uint16_t frequency_hz, uint32_t duration_ms);
void beep(uint32_t duration_ms);
void set_cluster_power(bool enabled);
bool cluster_power_enabled();
void delay_ms(uint32_t duration_ms);
void reboot_to_bootloader();

} // namespace rewrite_bsp
