#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <TCA6408.h>

#include <W5500lwIP.h>

#include "ClusterController.hpp"


using namespace QP;
using namespace CC;

namespace CC
{
namespace constants
{
constexpr pin_size_t led_pin = 25;
constexpr pin_size_t power_pin = 15;

// Serial Communication Interface
// constexpr pin_size_t serial_rx_pin = 17;
// constexpr pin_size_t serial_tx_pin = 16;

// Ethernet SPI Settings
constexpr pin_size_t ethernet_cs_pin = 17;
constexpr pin_size_t ethernet_int_pin = 21;

// Wire settings
constexpr pin_size_t sda_pin = 26;
constexpr pin_size_t scl_pin = 27;

constexpr pin_size_t cluster_address_reset_pin = 0;
constexpr pin_size_t cluster_address_interrupt_pin = 1;
constexpr TCA6408::DeviceAddress cluster_address_device_address = TCA6408::DEVICE_ADDRESS_0;

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

// Ethernet Communication Interface
Wiznet5500lwIP eth(constants::ethernet_cs_pin, SPI, constants::ethernet_int_pin);

static TwoWire & wire = Wire1;
static TCA6408 tca6408;

// Ethernet Communication Interface

//----------------------------------------------------------------------------
// Local functions
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
  return serial_communication_interface_stream.available();
}

uint8_t BSP::readSerialByte()
{
  return serial_communication_interface_stream.read();
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

bool BSP::initializeEthernet()
{
  if (eth.begin())
  {
    return true;
  }
  return false;
}

bool BSP::ethernetConnected()
{
  if (eth.connected())
  {
    return true;
  }
  return false;
}

void BSP::pollEthernet()
{
}

bool BSP::createEthernetServerConnection()
{
  return false;
}

void BSP::writeEthernetBinaryResponse(void * connection, uint8_t response[constants::byte_count_per_response_max], uint8_t response_byte_count)
{
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
