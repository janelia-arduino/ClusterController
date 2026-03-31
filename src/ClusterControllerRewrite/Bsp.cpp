#include "ClusterControllerRewrite/Bsp.hpp"

#include <Arduino.h>

namespace cluster_controller_rewrite
{
namespace
{

constexpr pin_size_t led_pin = LED_BUILTIN;
constexpr pin_size_t power_pin = 15;
constexpr pin_size_t tone_pin = 28;

constexpr pin_size_t ethernet_spi_rx_pin = 16;
constexpr pin_size_t ethernet_spi_csn_pin = 17;
constexpr pin_size_t ethernet_spi_sck_pin = 18;
constexpr pin_size_t ethernet_spi_tx_pin = 19;
constexpr pin_size_t ethernet_reset_pin = 20;
constexpr pin_size_t ethernet_int_pin = 21;

constexpr pin_size_t wire_sda_pin = 26;
constexpr pin_size_t wire_scl_pin = 27;
constexpr pin_size_t cluster_address_reset_pin = 0;
constexpr pin_size_t cluster_address_interrupt_pin = 1;

constexpr uint32_t heartbeat_interval_ms = 500;
constexpr uint16_t heartbeat_tone_hz = 1760;
constexpr uint16_t heartbeat_tone_duration_ms = 40;

} // namespace

void Bsp::setup()
{
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  pinMode(power_pin, OUTPUT);
  digitalWrite(power_pin, LOW);

  pinMode(tone_pin, OUTPUT);
  digitalWrite(tone_pin, LOW);

  pinMode(ethernet_spi_csn_pin, OUTPUT);
  digitalWrite(ethernet_spi_csn_pin, HIGH);
  pinMode(ethernet_reset_pin, OUTPUT);
  digitalWrite(ethernet_reset_pin, LOW);
  delay(10);
  digitalWrite(ethernet_reset_pin, HIGH);
  pinMode(ethernet_int_pin, INPUT_PULLUP);

  pinMode(cluster_address_reset_pin, OUTPUT);
  digitalWrite(cluster_address_reset_pin, HIGH);
  pinMode(cluster_address_interrupt_pin, INPUT_PULLUP);

  led_on_ = false;
  digitalWrite(led_pin, led_on_ ? HIGH : LOW);
  tone(tone_pin, heartbeat_tone_hz, 120);
  next_heartbeat_ms_ = millis() + heartbeat_interval_ms;
}

void Bsp::tick()
{
  const uint32_t now_ms = millis();
  if (static_cast<int32_t>(now_ms - next_heartbeat_ms_) < 0)
  {
    return;
  }

  led_on_ = !led_on_;
  digitalWrite(led_pin, led_on_ ? HIGH : LOW);
  tone(tone_pin, heartbeat_tone_hz, heartbeat_tone_duration_ms);
  next_heartbeat_ms_ = now_ms + heartbeat_interval_ms;
}

void Bsp::printBootBanner() const
{
  Serial.println();
  Serial.println("ClusterController rewrite checkpoint");
  Serial.printf("pins led/tone = %u/%u\n", led_pin, tone_pin);
  Serial.printf("pins eth cs/reset/int = %u/%u/%u\n",
                ethernet_spi_csn_pin,
                ethernet_reset_pin,
                ethernet_int_pin);
  Serial.printf("pins wire sda/scl addr_reset/addr_int = %u/%u/%u/%u\n",
                wire_sda_pin,
                wire_scl_pin,
                cluster_address_reset_pin,
                cluster_address_interrupt_pin);
}

} // namespace cluster_controller_rewrite
