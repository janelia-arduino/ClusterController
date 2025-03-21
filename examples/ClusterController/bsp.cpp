#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <EthernetCompat.h>
#include <TCA6408.h>

#include "bsp.hpp"
#include "ClusterController.hpp"


using namespace QP;

namespace CC
{
namespace constants
{
constexpr uint8_t led_pin = 25;
constexpr uint8_t power_pin = 15;

// Serial Communication Interface
constexpr uint8_t serial_rx_pin = 17;
constexpr uint8_t serial_tx_pin = 16;

// SPI Settings
constexpr uint32_t spi_clock_speed = 5000000;

// Wire settings
constexpr uint8_t sda_pin = 26;
constexpr uint8_t scl_pin = 27;

constexpr uint8_t cluster_address_reset_pin = 0;
constexpr uint8_t cluster_address_interrupt_pin = 1;
constexpr TCA6408::DeviceAddress cluster_address_device_address = TCA6408::DEVICE_ADDRESS_0;

} // namespace constants
} // namespace CC

//----------------------------------------------------------------------------
// QS facilities

// un-comment if QS instrumentation needed
#define QS_ON

static QP::QSpyId const l_BSP_ID = { 1U }; // QSpy source ID

//----------------------------------------------------------------------------
// Static global variables
static Ticker system_clock;

// Serial Communication Interface
// HardwareSerial & SERIAL_COMMUNICATION_INTERFACE_STREAM = Serial;
// HardwareSerial & QS_SERIAL_STREAM = Serial1;
static HardwareSerial & SERIAL_COMMUNICATION_INTERFACE_STREAM = Serial1;
static HardwareSerial & QS_SERIAL_STREAM = Serial;

static ArduinoWiznet5500lwIP Ethernet(17, SPI, 21);
static WiFiServer ethernet_server;
static TwoWire & wire = Wire1;
static TCA6408 tca6408;

//----------------------------------------------------------------------------
// Local functions
void ipAddressToString(IPAddress ip_address, char * ip_address_str)
{
  sprintf(ip_address_str,"%u.%u.%u.%u", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
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

#ifdef QS_ON
  QS_INIT(nullptr);

  // output QS dictionaries...
  QS_OBJ_DICTIONARY(&l_BSP_ID);

  // setup the QS filters...
  QS_GLB_FILTER(QP::QS_SM_RECORDS); // state machine records
  QS_GLB_FILTER(QP::QS_AO_RECORDS); // active object records
  QS_GLB_FILTER(QP::QS_UA_RECORDS); // all user records
#endif
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
  CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.setRx();
  CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.setTx();
  CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.begin(CC::constants::serial_baud_rate);
  CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.setTimeout(CC::constants::serial_timeout);
  return true;
}

bool BSP::pollSerialCommand()
{
  return CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.available();
}

uint8_t BSP::readSerialByte()
{
  return CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.read();
}

void BSP::readSerialStringCommand(char * command_str, char first_char)
{
  char command_tail[CC::constants::string_command_length_max];
  size_t chars_read = CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.readBytesUntil(CC::constants::command_termination_character,
    command_tail, CC::constants::string_command_length_max - 1);
  command_tail[chars_read] = '\0';
  command_str[0] = first_char;
  command_str[1] = '\0';
  strcat(command_str, command_tail);
}

void BSP::writeSerialStringResponse(char * response)
{
  CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.println(response);
}

void BSP::initializeEthernet()
{
  SPI.setRX(16);
  SPI.setCS(17);
  SPI.setSCK(18);
  SPI.setTX(19);
}

bool BSP::beginEthernet()
{
  uint8_t cluster_address = readClusterAddress();
  uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, cluster_address };
  return Ethernet.begin(mac);
  // // IPAddress ip(192, 168, 10, cluster_address);

  // // if (Ethernet.begin(mac, ip))
  // if (Ethernet.begin(mac))
  // {
  //   CC::AO_EthernetCommandInterface->POST(&ethernetInitializedEvt, &l_BSP_ID);
  // }
  // else
  // {
  //   if (not Ethernet.hardwareStatus())
  //   {
  //     Serial.println("No Ethernet hardware detected. Check pinouts, wiring.");
  //   }
  //   else if (not Ethernet.linkStatus())
  //   {
  //     Serial.println("No Ethernet link detected. Check cable connections.");
  //   }
  //   else
  //   {
  //     Serial.println("Ethernet not initialized with mac and IP address.");
  //   }
  // }
}

bool BSP::beginEthernetServer()
{
  ethernet_server.begin(CC::constants::ethernet_server_port);
  return true;
}

bool BSP::pollEthernetCommand()
{
  WiFiClient client = ethernet_server.accept();
  if (client && client.available())
  {
    return true;
    // String command = client.readStringUntil('\n');
    // String response = processCommandString(command);
    // client.write(response.c_str());
  }
  return false;
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

#ifdef QS_ON

  // transmit QS outgoing data (QS-TX)
  uint16_t len = CC::constants::QS_SERIAL_STREAM.availableForWrite();
  if (len > 0U)
  { // any space available in the output buffer?
    uint8_t const *buf = QS::getBlock(&len);
    if (buf)
    {
      CC::constants::QS_SERIAL_STREAM.write(buf, len); // asynchronous and non-blocking
    }
  }

  // receive QS incoming data (QS-RX)
  len = CC::constants::QS_SERIAL_STREAM.available();
  if (len > 0U)
  {
    do
    {
      QP::QS::rxPut(CC::constants::QS_SERIAL_STREAM.read());
    } while (--len > 0U);
    QS::rxParse();
  }

#endif // QS_ON

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
#ifdef QS_ON

//............................................................................
bool QP::QS::onStartup(void const * arg)
{
  static uint8_t qsTxBuf[1024]; // buffer for QS transmit channel (QS-TX)
  static uint8_t qsRxBuf[128];  // buffer for QS receive channel (QS-RX)
  initBuf  (qsTxBuf, sizeof(qsTxBuf));
  rxInitBuf(qsRxBuf, sizeof(qsRxBuf));
  CC::constants::QS_SERIAL_STREAM.begin(115200); // run serial port at 115200 baud rate
  return true; // return success
}
//............................................................................
void QP::QS::onCommand(uint8_t cmdId, uint32_t param1,
  uint32_t param2, uint32_t param3)
{
}

#endif // QS_ON

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
#ifdef QS_ON
  uint16_t len = 0xFFFFU; // big number to get as many bytes as available
  uint8_t const *buf = QS::getBlock(&len); // get continguous block of data
  while (buf != nullptr)
  { // data available?
    CC::constants::QS_SERIAL_STREAM.write(buf, len); // might poll until all bytes fit
    len = 0xFFFFU; // big number to get as many bytes as available
    buf = QS::getBlock(&len); // try to get more data
  }
  CC::constants::QS_SERIAL_STREAM.flush(); // wait for the transmission of outgoing data to complete
#endif // QS_ON
}
//............................................................................
void QP::QS::onReset()
{
  //??? TBD
}
