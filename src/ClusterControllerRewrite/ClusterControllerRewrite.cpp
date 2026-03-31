#include "ClusterControllerRewrite.hpp"

#include <Arduino.h>

namespace
{
constexpr pin_size_t rewrite_led_pin = 25;
constexpr pin_size_t rewrite_tone_pin = 28;
bool rewrite_led_state = false;
}

namespace RewriteArduinoInterface
{

void setup()
{
  pinMode(rewrite_led_pin, OUTPUT);
  pinMode(rewrite_tone_pin, OUTPUT);
  digitalWrite(rewrite_led_pin, LOW);
  noTone(rewrite_tone_pin);
}

void loop()
{
  rewrite_led_state = !rewrite_led_state;
  digitalWrite(rewrite_led_pin, rewrite_led_state ? HIGH : LOW);
  tone(rewrite_tone_pin, rewrite_led_state ? 1760 : 880, 60);
  delay(500);
}

} // namespace RewriteArduinoInterface
