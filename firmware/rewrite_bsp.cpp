#include "rewrite_bsp.hpp"

#include <Arduino.h>

namespace rewrite_bsp
{
namespace
{

constexpr pin_size_t led_pin = 25;
constexpr pin_size_t tone_pin = 28;
constexpr pin_size_t power_pin = 15;
constexpr uint16_t beep_frequency_hz = 5000;
bool command_led_override = false;
bool power_enabled = false;

} // namespace

void setup()
{
  pinMode(led_pin, OUTPUT);
  pinMode(tone_pin, OUTPUT);
  pinMode(power_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  digitalWrite(power_pin, LOW);
  power_enabled = false;
  noTone(tone_pin);
}

void set_status_led(const bool enabled)
{
  if (command_led_override) {
    return;
  }
  digitalWrite(led_pin, enabled ? HIGH : LOW);
}

void set_command_led(const bool enabled)
{
  command_led_override = true;
  digitalWrite(led_pin, enabled ? HIGH : LOW);
}

void play_status_tone(const uint16_t frequency_hz, const uint32_t duration_ms)
{
  tone(tone_pin, frequency_hz, duration_ms);
}

void beep(const uint32_t duration_ms)
{
  tone(tone_pin, beep_frequency_hz, duration_ms);
}

void set_cluster_power(const bool enabled)
{
  digitalWrite(power_pin, enabled ? HIGH : LOW);
  power_enabled = enabled;
}

bool cluster_power_enabled()
{
  return power_enabled;
}

void delay_ms(const uint32_t duration_ms)
{
  delay(duration_ms);
}

} // namespace rewrite_bsp
