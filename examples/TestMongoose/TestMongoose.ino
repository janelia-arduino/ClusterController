#include <SPI.h>
#include "mongoose_glue.h"

#define SS_PIN 17            // Slave select pin
#define LED_PIN LED_BUILTIN  // LED pin

struct mg_tcpip_spi spi = {
    NULL,  // SPI metadata
    [](void *) { digitalWrite(SS_PIN, LOW); SPI.beginTransaction(SPISettings()); },
    [](void *) { digitalWrite(SS_PIN, HIGH); SPI.endTransaction(); },
    [](void *, uint8_t c) { return SPI.transfer(c); }, // Execute transaction
};
struct mg_tcpip_if mif = {.mac = {2, 0, 1, 2, 3, 5}};  // network interface

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(50);

  pinMode(SS_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  SPI.setRX(16);
  SPI.setSCK(18);
  SPI.setTX(19);
  SPI.begin();

  // Set logging function to serial print
  mg_log_set_fn([](char ch, void *) { Serial.print(ch); }, NULL);
  mg_log_set(MG_LL_DEBUG);

  mongoose_init();

  // Initialise built-in TCP/IP stack with W5500 driver
  mif.driver = &mg_tcpip_driver_w5500;
  mif.driver_data = &spi;
  mg_tcpip_init(&g_mgr, &mif);
}

void loop() {
  mongoose_poll();
}
