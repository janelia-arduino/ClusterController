#include <TMC51X0.hpp>


const uint8_t TONE_PIN = 28;
const uint16_t TONE_DURATION = 500;
const uint16_t TONE_FREQUENCY_MIN = 1000;
const uint16_t TONE_FREQUENCY_MAX = 10000;
const uint16_t TONE_FREQUENCY_INC = 1000;

const uint16_t LOOP_DELAY = 5000;

uint16_t tone_frequency;

void setup()
{
  tone_frequency = TONE_FREQUENCY_MIN;
}

void loop()
{
  tone(TONE_PIN, tone_frequency, TONE_DURATION);

  tone_frequency += TONE_FREQUENCY_INC;
  if (tone_frequency > TONE_FREQUENCY_MAX)
  {
    tone_frequency = TONE_FREQUENCY_MIN;
  }

  delay(LOOP_DELAY);
}
