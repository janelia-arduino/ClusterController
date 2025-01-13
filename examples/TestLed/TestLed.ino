#include <TMC51X0.hpp>


const uint8_t LED_PIN = 25;

const uint16_t DELAY = 1000;
bool led_on;

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  led_on = false;
}

void loop()
{
  Serial.print("LED On: ");
  Serial.println(led_on);
  delay(DELAY);

  led_on = !led_on;
  if (led_on)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}
