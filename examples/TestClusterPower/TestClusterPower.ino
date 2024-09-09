#include <TMC51X0.hpp>


const uint8_t ENABLE_VCC_PIN = 22;
const uint8_t ENABLE_VCC_POLARITY = HIGH;

const uint16_t DELAY = 10000;
bool enabled;

void setup()
{
  pinMode(ENABLE_VCC_PIN, OUTPUT);
  digitalWrite(ENABLE_VCC_PIN, LOW);
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
    digitalWrite(ENABLE_VCC_PIN, HIGH);
  }
  else
  {
    digitalWrite(ENABLE_VCC_PIN, LOW);
  }
}
