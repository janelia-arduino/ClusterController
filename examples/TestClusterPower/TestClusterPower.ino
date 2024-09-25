#include <TMC51X0.hpp>


const uint8_t ENABLE_POWER_PIN = 15;

const uint16_t DELAY = 5000;
bool enabled;

void setup()
{
  pinMode(ENABLE_POWER_PIN, OUTPUT);
  digitalWrite(ENABLE_POWER_PIN, LOW);
  enabled = false;
}

void loop()
{
  Serial.print("Enabled: ");
  Serial.println(enabled);
  delay(DELAY);

  enabled = !enabled;
  if (enabled)
  {
    digitalWrite(ENABLE_POWER_PIN, HIGH);
  }
  else
  {
    digitalWrite(ENABLE_POWER_PIN, LOW);
  }
}
