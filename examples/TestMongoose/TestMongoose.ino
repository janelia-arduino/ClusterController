#include <SPI.h>
#include "mongoose_glue.h"

#define SS_PIN 17            // Slave select pin
#define LED_PIN LED_BUILTIN  // LED pin

SPISettings ethernet_spi_settings(4000000,
  MSBFIRST,
  SPI_MODE0);

struct mg_tcpip_spi spi = {
    NULL,  // SPI metadata
    [](void *) { SPI.beginTransaction(ethernet_spi_settings); digitalWrite(SS_PIN, LOW); },
    [](void *) { digitalWrite(SS_PIN, HIGH); SPI.endTransaction(); },
    [](void *, uint8_t c) { return SPI.transfer(c); }, // Execute transaction
};

// Construct MAC address from the unique board ID
#include "pico/unique_id.h"
static inline void genmac(unsigned char *mac) {
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);
  mac[0] = 2;
  memcpy(&mac[1], &board_id.id[3], 5);
}
struct mg_tcpip_if mif;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(50);

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
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
  genmac(mif.mac);
  // mif.enable_dhcp_client = false;
  // mif.ip = MG_IPV4(192, 168, 10, 55);
  // mif.gw = MG_IPV4(192, 168, 10, 1);
  // mif.mask = MG_IPV4(255, 255, 255, 0);
  mif.enable_dhcp_client = true;
  mif.ip = MG_IPV4(0, 0, 0, 0);
  mif.gw = MG_IPV4(0, 0, 0, 0);
  mif.mask = MG_IPV4(0, 0, 0, 0);
  mif.driver = &mg_tcpip_driver_w5500;
  mif.driver_data = &spi;
  mg_tcpip_init(&g_mgr, &mif);
}

void loop() {
  mongoose_poll();
}
