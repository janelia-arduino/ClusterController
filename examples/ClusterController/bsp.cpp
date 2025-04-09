#include <Arduino.h>
#include <SPI.h>
#include "mongoose.h"
#include <Ticker.h>
#include <TCA6408.h>
#include <TMC51X0.hpp>

#include "ClusterController.hpp"


using namespace QP;
using namespace CC;

namespace CC
{
namespace constants
{
constexpr pin_size_t led_pin = LED_BUILTIN;
constexpr pin_size_t power_pin = 15;
constexpr pin_size_t tone_pin = 28;

// Serial Communication Interface
// constexpr pin_size_t serial_rx_pin = 17;
// constexpr pin_size_t serial_tx_pin = 16;

// Ethernet
constexpr pin_size_t ethernet_spi_rx_pin = 16;
constexpr pin_size_t ethernet_spi_csn_pin = 17;
constexpr pin_size_t ethernet_spi_sck_pin = 18;
constexpr pin_size_t ethernet_spi_tx_pin = 19;
constexpr pin_size_t ethernet_reset_pin = 20;
constexpr pin_size_t ethernet_int_pin = 21;

// Wire settings
constexpr pin_size_t sda_pin = 26;
constexpr pin_size_t scl_pin = 27;

constexpr pin_size_t cluster_address_reset_pin = 0;
constexpr pin_size_t cluster_address_interrupt_pin = 1;
constexpr TCA6408::DeviceAddress cluster_address_device_address = TCA6408::DEVICE_ADDRESS_0;

// Prism Settings
constexpr uint8_t prism_count = 7;
constexpr pin_size_t prism_spi_sck_pin = 10;
constexpr pin_size_t prism_spi_tx_pin = 11;
constexpr pin_size_t prism_spi_rx_pin = 12;
constexpr pin_size_t prism_spi_csn_pins[prism_count] = {14, 8, 7, 6, 5, 4, 3};
constexpr uint32_t prism_spi_clock_rate = 1000000;

const tmc51x0::ConverterParameters converter_parameters =
{
  16, // clock_frequency_mhz
  4881 // microsteps_per_real_unit
};
// external clock is 16MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49 ~= 4881
// one "real unit" in this example is one millimeters of linear travel

const tmc51x0::DriverParameters driver_parameters_real =
{
  100, // global_current_scaler (percent)
  50, // run_current (percent)
  20, // hold_current (percent)
  0, // hold_delay (percent)
  15, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::REVERSE, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  10, // stealth_chop_threshold (millimeters/s)
  true, // stealth_chop_enabled
  50, // cool_step_threshold (millimeters/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  90, // high_velocity_threshold (millimeters/s)
  false, // high_velocity_fullstep_enabled
  false, // high_velocity_chopper_switch_enabled
  1, // stall_guard_threshold
  false, // stall_guard_filter_enabled
  true, // short_to_ground_protection_enabled
  3, // enabled_toff
  tmc51x0::CLOCK_CYCLES_36, // comparator_blank_time
  37, // dc_time
  3 // dc_stall_guard_threshold
};

const tmc51x0::ControllerParameters controller_parameters_real =
{
  tmc51x0::POSITION, // ramp_mode
  tmc51x0::HARD, // stop_mode
  20, // max_velocity (millimeters/s)
  2, // max_acceleration ((millimeters/s)/s)
  1, // start_velocity (millimeters/s)
  5, // stop_velocity (millimeters/s)
  10, // first_velocity (millimeters/s)
  10, // first_acceleration ((millimeters/s)/s)
  20, // max_deceleration ((millimeters/s)/s)
  25, // first_deceleration ((millimeters/s)/s)
  0, // zero_wait_duration (milliseconds)
  false // stall_stop_enabled
};

const tmc51x0::HomeParameters home_parameters_real =
{
  50, // run_current (percent)
  20, // hold_current (percent)
  -1000, // target_position (millimeters)
  20, // velocity (millimeters/s)
  2, // acceleration ((millimeters/s)/s)
  100 // zero_wait_duration (milliseconds)
};

const tmc51x0::StallParameters stall_parameters_real =
{
  tmc51x0::COOL_STEP, // stall_mode
  10, // stall_guard_threshold
  15 // cool_step_threshold (millimeters/s)
};
} // namespace constants
} // namespace CC

//----------------------------------------------------------------------------
// QS facilities

static QP::QSpyId const l_BSP_ID = { 1U }; // QSpy source ID

//----------------------------------------------------------------------------
// Static global variables
static Ticker system_clock;

// Serial Communication Interface
static SerialUART & serial_communication_interface_stream = Serial1;
static SerialUSB & qs_serial_stream = Serial;

static TwoWire & wire = Wire1;
static TCA6408 tca6408;

// Ethernet Communication Interface
SPIClassRP2040 & ethernet_spi = SPI;
SPISettings ethernet_spi_settings = SPISettings();
struct mg_mgr mgr;
struct mg_tcpip_if mif;
static const char *s_lsn = "tcp://0.0.0.0:7777";

// Log
static char log_str[constants::string_log_length_max];
static uint16_t log_str_pos = 0;

// Prism Settings
TMC51X0 prisms[constants::prism_count];
tmc51x0::DriverParameters driver_parameters_chip;
tmc51x0::ControllerParameters controller_parameters_chip;
tmc51x0::HomeParameters home_parameters_chip;
tmc51x0::StallParameters stall_parameters_chip;

//----------------------------------------------------------------------------
// Local functions
struct mg_tcpip_spi mongoose_spi = {
    NULL,  // SPI metadata
    [](void *) { digitalWriteFast(constants::ethernet_spi_csn_pin, LOW); ethernet_spi.beginTransaction(ethernet_spi_settings); },
    [](void *) { digitalWriteFast(constants::ethernet_spi_csn_pin, HIGH); ethernet_spi.endTransaction(); },
    [](void *, uint8_t c) { return ethernet_spi.transfer(c); }, // Execute transaction
};

// // Construct MAC address from the unique board ID
#include "pico/unique_id.h"
static inline void genmac(unsigned char *mac) {
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);
  mac[0] = 2;
  memcpy(&mac[1], &board_id.id[3], 5);
}

// Used by Mongoose for time tracking
uint64_t mg_millis(void) {
  return millis();
}

// Used by Mongoose to generate random data
bool mg_random(void *buf, size_t len) {  // For TLS
  uint8_t *p = (uint8_t *) buf;
  while (len--) *p++ = (unsigned char) (rand() & 255);
  return true;
}

// Crude function to get available RAM, for quick profiling
extern "C" char *sbrk(int);
extern char *__brkval;
int getFreeRAM() {
  char top;
#ifdef __arm__
  return &top - (char *) sbrk(0);
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif
}
int getConst() {
  return 77;
}

void addressInterruptCallback()
{
  // CC::AO_SerialCommandInterface->POST(&activateSerialCommandInterfaceEvt, &l_BSP_ID);
}

//----------------------------------------------------------------------------
// BSP functions

void BSP::init()
{
  // initialize the hardware used in this sketch...
  // NOTE: interrupts are configured and started later in QF::onStartup()

  pinMode(CC::constants::led_pin, OUTPUT);
  ledOff();

  QS_OBJ_DICTIONARY(&l_BSP_ID);
}

void BSP::ledOff()
{
  digitalWriteFast(CC::constants::led_pin, LOW);
}

void BSP::ledOn()
{
  digitalWriteFast(CC::constants::led_pin, HIGH);
}

void BSP::initializeWatchdog()
{
  rp2040.wdt_begin(CC::constants::watchdog_delay_ms);
}

void BSP::feedWatchdog()
{
  rp2040.wdt_reset();
}

void BSP::initializeCluster()
{
  pinMode(CC::constants::power_pin, OUTPUT);
  powerOff();

  pinMode(CC::constants::tone_pin, OUTPUT);

  wire.setSDA(CC::constants::sda_pin);
  wire.setSCL(CC::constants::scl_pin);

  tca6408.setup(wire, CC::constants::cluster_address_device_address);
  tca6408.setResetPin(CC::constants::cluster_address_reset_pin);

  tca6408.attachInterrupt(CC::constants::cluster_address_interrupt_pin, addressInterruptCallback);
}

uint8_t BSP::readClusterAddress()
{
  return tca6408.readInputRegister();
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void BSP::beep(uint16_t duration_ms)
{
  uint8_t cluster_address = readClusterAddress();
  uint16_t frequency = map(cluster_address,
    constants::cluster_address_min,
    constants::cluster_address_max,
    constants::beep_frequency_min,
    constants::beep_frequency_max);
  tone(constants::tone_pin, frequency, duration_ms);
}

void BSP::powerOff()
{
  digitalWriteFast(CC::constants::power_pin, LOW);
}

void BSP::powerOn()
{
  digitalWriteFast(CC::constants::power_pin, HIGH);
}

bool BSP::beginSerial()
{
  // serial_communication_interface_stream.setRX(CC::constants::serial_rx_pin);
  // serial_communication_interface_stream.setTX(CC::constants::serial_tx_pin);
  // serial_communication_interface_stream.begin(CC::constants::serial_baud_rate);
  // serial_communication_interface_stream.setTimeout(CC::constants::serial_timeout);
  return true;
}

bool BSP::pollSerialCommand()
{
  return false;
  // return serial_communication_interface_stream.available();
}

uint8_t BSP::readSerialByte()
{
  return 0;
  // return serial_communication_interface_stream.read();
}

void BSP::readSerialStringCommand(char * command_str, char first_char)
{
  char command_tail[CC::constants::string_command_length_max];
  size_t chars_read = serial_communication_interface_stream.readBytesUntil(CC::constants::command_termination_character,
    command_tail, CC::constants::string_command_length_max - 1);
  command_tail[chars_read] = '\0';
  command_str[0] = first_char;
  command_str[1] = '\0';
  strcat(command_str, command_tail);
}

void BSP::writeSerialStringResponse(char * response)
{
  serial_communication_interface_stream.println(response);
}

void log_fn(char ch, void *param)
{
  if ((ch == '\n') || (log_str_pos == (constants::string_log_length_max - 1)))
  {
    log_str[log_str_pos] = 0;
    QS_BEGIN_ID(ETHERNET_LOG, AO_EthernetCommandInterface->m_prio)
      QS_STR(log_str);
    QS_END()
    log_str[0] = 0;
    log_str_pos = 0;
  }
  else if (ch != '\r')
  {
    log_str[log_str_pos++] = ch;
  }
}

static void http_ev_handler(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    if (mg_match(hm->uri, mg_str("/api/led/on"), NULL)) {
      //BSP::ledOn();
      //mg_http_reply(c, 200, "", "{%m: %d}\n", MG_ESC("led"), digitalRead(constants::led_pin));
    } else if (mg_match(hm->uri, mg_str("/api/led/off"), NULL)) {
      // BSP::ledOff();
      //mg_http_reply(c, 200, "", "{%m: %d}\n", MG_ESC("led"), digitalRead(constants::led_pin));
    } else {
      mg_http_reply(c, 200, "", "ok, free RAM: %u\n", getFreeRAM());
    }
  }
}

bool BSP::initializeEthernet()
{
  pinMode(constants::ethernet_spi_csn_pin, OUTPUT);

  ethernet_spi.setRX(constants::ethernet_spi_rx_pin);
  ethernet_spi.setSCK(constants::ethernet_spi_sck_pin);
  ethernet_spi.setTX(constants::ethernet_spi_tx_pin);
  ethernet_spi.begin();

  mg_mgr_init(&mgr);

  mg_log_set(MG_LL_INFO);
  mg_log_set_fn(log_fn, 0);

  // Initialise built-in TCP/IP stack with W5500 driver
  genmac(mif.mac);
  mif.enable_dhcp_client = false;
  mif.ip = MG_IPV4(192, 168, 10, readClusterAddress());
  mif.gw = MG_IPV4(192, 168, 10, 1);
  mif.mask = MG_IPV4(255, 255, 255, 0);
  mif.driver = &mg_tcpip_driver_w5500;
  mif.driver_data = &mongoose_spi;
  mg_tcpip_init(&mgr, &mif);

  // Setup HTTP listener. Respond "ok" on any HTTP request
  mg_http_listen(&mgr, "http://0.0.0.0:80", http_ev_handler, NULL);

  return true;
}

void BSP::pollEthernet()
{
  mg_mgr_poll(&mgr, 1);
}

void sfn(struct mg_connection *c, int ev, void *ev_data)
{
  if (ev == MG_EV_OPEN && c->is_listening == 1)
  {
    MG_INFO(("SERVER is listening"));
  }
  else if (ev == MG_EV_ACCEPT)
  {
    MG_INFO(("SERVER accepted a connection"));
  }
  else if (ev == MG_EV_READ)
  {
    struct mg_iobuf *r = &c->recv;
    MG_INFO(("SERVER got data: %lu bytes", r->len));

    static EthernetCommandEvt ethernetCommandEvt = {ETHERNET_COMMAND_AVAILABLE_SIG, 0U, 0U};
    ethernetCommandEvt.connection = c;
    ethernetCommandEvt.binary_command = r->buf;
    ethernetCommandEvt.binary_command_byte_count = r->len;
    QF::PUBLISH(&ethernetCommandEvt, &l_BSP_ID);
  }
  else if (ev == MG_EV_WRITE)
  {
    MG_INFO(("MG_EV_WRITE"));
  }
  else if (ev == MG_EV_CLOSE)
  {
    MG_INFO(("SERVER disconnected"));
  }
  else if (ev == MG_EV_ERROR)
  {
    MG_INFO(("SERVER error: %s", (char *) ev_data));
  }
  else if (ev == MG_EV_POLL)
  {
  }
  else
  {
    MG_INFO(("event %lu", ev));
  }
}

bool BSP::createEthernetServerConnection()
{
  struct mg_connection *c = mg_listen(&mgr, s_lsn, sfn, NULL);
  if (c == NULL)
  {
    MG_INFO(("SERVER cannot open a connection"));
    return false;
  }
  return true;
}

void BSP::writeEthernetBinaryResponse(void * connection, uint8_t response[constants::byte_count_per_response_max], uint8_t response_byte_count)
{
  struct mg_connection * c = (struct mg_connection *)connection;
  struct mg_iobuf *r = &c->recv;
  mg_send(c, response, response_byte_count);
  r->len = 0;
}

//----------------------------------------------------------------------------
// QF callbacks...

//
// NOTE: The usual source of system clock tick in ARM Cortex-M (SysTick timer)
// is aready used by the Arduino library. Therefore, this code uses a different
// hardware timer for providing the system clock tick.
//
// NOTE: You can re-define the macros to use a different ATSAM timer/channel.
//

#define TIMER_HANDLER   T1_Handler

// interrupts.................................................................
void TIMER_HANDLER()
{
  QF::TICK_X(0, &l_BSP_ID); // process time events for tick rate 0
}
//............................................................................
void QF::onStartup()
{
  // configure the timer-counter channel........
  system_clock.attach_ms(CC::constants::milliseconds_per_second / CC::constants::ticks_per_second,
    TIMER_HANDLER);
  // ...
}
//............................................................................
void QV::onIdle()
{ // called with interrupts DISABLED
#ifdef NDEBUG
  // Put the CPU and peripherals to the low-power mode. You might
  // need to customize the clock management for your application,
  // see the datasheet for your particular MCU.
  QV_CPU_SLEEP();  // atomically go to sleep and enable interrupts
#else
  QF_INT_ENABLE(); // simply re-enable interrupts

  // transmit QS outgoing data (QS-TX)
  uint16_t len = qs_serial_stream.availableForWrite();
  if (len > 0U)
  { // any space available in the output buffer?
    uint8_t const *buf = QS::getBlock(&len);
    if (buf)
    {
      qs_serial_stream.write(buf, len); // asynchronous and non-blocking
    }
  }

  // receive QS incoming data (QS-RX)
  len = qs_serial_stream.available();
  if (len > 0U)
  {
    do
    {
      QP::QS::rxPut(qs_serial_stream.read());
    } while (--len > 0U);
    QS::rxParse();
  }
#endif
}
//............................................................................
extern "C" Q_NORETURN Q_onAssert(char const * const module, int location)
{
  //
  // NOTE: add here your application-specific error handling
  //
  (void)module;
  (void)location;

  QF_INT_DISABLE(); // disable all interrupts
  BSP::ledOn();  // turn the LED on
  for (;;)
  { // freeze in an endless loop for now...
  }
}

//----------------------------------------------------------------------------
// QS callbacks...
//............................................................................
bool QP::QS::onStartup(void const * arg)
{
  static uint8_t qsTxBuf[2048]; // buffer for QS transmit channel (QS-TX)
  static uint8_t qsRxBuf[1024];  // buffer for QS receive channel (QS-RX)
  initBuf  (qsTxBuf, sizeof(qsTxBuf));
  rxInitBuf(qsRxBuf, sizeof(qsRxBuf));
  qs_serial_stream.begin(115200); // run serial port at 115200 baud rate
  return true; // return success
}
//............................................................................
void QP::QS::onCommand(uint8_t cmdId, uint32_t param1,
  uint32_t param2, uint32_t param3)
{
}

//............................................................................
void QP::QS::onCleanup()
{
}
//............................................................................
QP::QSTimeCtr QP::QS::onGetTime()
{
  return millis();
}
//............................................................................
void QP::QS::onFlush()
{
  uint16_t len = 0xFFFFU; // big number to get as many bytes as available
  uint8_t const *buf = QS::getBlock(&len); // get continguous block of data
  while (buf != nullptr)
  { // data available?
    qs_serial_stream.write(buf, len); // might poll until all bytes fit
    len = 0xFFFFU; // big number to get as many bytes as available
    buf = QS::getBlock(&len); // try to get more data
  }
  qs_serial_stream.flush(); // wait for the transmission of outgoing data to complete
}
//............................................................................
void QP::QS::onReset()
{
  rp2040.restart();
}
