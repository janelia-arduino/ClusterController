#include "rewrite_heartbeat.hpp"

#include <Arduino.h>

namespace rewrite_heartbeat
{
namespace
{

constexpr pin_size_t led_pin = 25;
constexpr pin_size_t tone_pin = 28;
bool led_state = false;

} // namespace

void setup()
{
  pinMode(led_pin, OUTPUT);
  pinMode(tone_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  noTone(tone_pin);
}

void loop()
{
  led_state = !led_state;
  digitalWrite(led_pin, led_state ? HIGH : LOW);
  tone(tone_pin, led_state ? 1760 : 880, 60);
  delay(500);
}

} // namespace rewrite_heartbeat
