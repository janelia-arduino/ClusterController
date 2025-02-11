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
HardwareSerial & SERIAL_COMMUNICATION_INTERFACE_STREAM = Serial;
HardwareSerial & QS_SERIAL_STREAM = Serial;
constexpr uint32_t SERIAL_COMMUNICATION_INTERFACE_BAUD_RATE = 115200;
constexpr uint16_t SERIAL_COMMUNICATION_INTERFACE_TIMEOUT = 100;

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
//#define QS_ON

static QP::QSpyId const l_TIMER_ID = { 0U }; // QSpy source ID

//----------------------------------------------------------------------------
// Static global variables
static Ticker system_clock;
static Ticker test_timer;

static QEvt const activateSerialCommandInterfaceEvt = { CC::ACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const activateEthernetCommandInterfaceEvt = { CC::ACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const deactivateSerialCommandInterfaceEvt = { CC::DEACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const deactivateEthernetCommandInterfaceEvt = { CC::DEACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const serialReadyEvt = { CC::SERIAL_READY_SIG, 0U, 0U};

static QEvt const ethernetInitializedEvt = { CC::ETHERNET_INITIALIZED_SIG, 0U, 0U};
static QEvt const ethernetServerInitializedEvt = { CC::ETHERNET_SERVER_INITIALIZED_SIG, 0U, 0U};

static CC::CommandEvt const resetEvt = { CC::RESET_SIG, 0U, 0U};
static CC::CommandEvt const powerOnEvt = { CC::POWER_ON_SIG, 0U, 0U};
static CC::CommandEvt const powerOffEvt = { CC::POWER_OFF_SIG, 0U, 0U};

static ArduinoWiznet5500lwIP Ethernet(17, SPI, 21);
static WiFiServer server;
static TwoWire & wire = Wire1;
static TCA6408 tca6408;

//----------------------------------------------------------------------------
// Local functions
String ipAddressToString(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3]);
}

String processCommandString(String command)
{
  command.trim();
  String response = command;
  if (command.equalsIgnoreCase("RESET"))
  {
    QF::PUBLISH(&resetEvt, &l_TIMER_ID);
  }
  if (command.equalsIgnoreCase("LED_ON"))
  {
    BSP::ledOn();
  }
  else if (command.equalsIgnoreCase("LED_OFF"))
  {
    BSP::ledOff();
  }
  else if (command.equalsIgnoreCase("POWER_ON"))
  {
    QF::PUBLISH(&powerOnEvt, &l_TIMER_ID);
  }
  else if (command.equalsIgnoreCase("POWER_OFF"))
  {
    QF::PUBLISH(&powerOffEvt, &l_TIMER_ID);
  }
  else if (command.equalsIgnoreCase("EHS"))
  {
    response = String(Ethernet.hardwareStatus());
  }
  else if (command.equalsIgnoreCase("ELS"))
  {
    response = String(Ethernet.linkStatus());
  }
  else if (command.equalsIgnoreCase("GET_IP_ADDRESS"))
  {
    response = ipAddressToString(Ethernet.localIP());
  }
  else if (command.equalsIgnoreCase("GET_CLUSTER_ADDRESS"))
  {
    response = String(tca6408.readInputRegister());
  }
  else if (command.equalsIgnoreCase("GET_ADDRESSES"))
  {
    response = String(tca6408.readInputRegister());
    response.concat(" ");
    response.concat(ipAddressToString(Ethernet.localIP()));
  }
  return response;
}

void addressInterruptCallback()
{
  // CC::AO_SerialCommandInterface->POST(&activateSerialCommandInterfaceEvt, &l_TIMER_ID);
}

//----------------------------------------------------------------------------
// BSP functions

void BSP::init()
{
  // initialize the hardware used in this sketch...
  // NOTE: interrupts are configured and started later in QF::onStartup()

  Serial.begin(CC::constants::SERIAL_COMMUNICATION_INTERFACE_BAUD_RATE);

  pinMode(CC::constants::led_pin, OUTPUT);
  ledOff();

#ifdef QS_ON
  QS_INIT(nullptr);

  // output QS dictionaries...
  QS_OBJ_DICTIONARY(&l_TIMER_ID);

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

void BSP::activateCommandInterfaces()
{
#ifndef QS_ON
  CC::AO_SerialCommandInterface->POST(&activateSerialCommandInterfaceEvt, &l_TIMER_ID);
#endif

  CC::AO_EthernetCommandInterface->POST(&activateEthernetCommandInterfaceEvt, &l_TIMER_ID);
}

void BSP::deactivateCommandInterfaces()
{
#ifndef QS_ON
  CC::AO_SerialCommandInterface->POST(&deactivateSerialCommandInterfaceEvt, &l_TIMER_ID);
#endif

  CC::AO_EthernetCommandInterface->POST(&deactivateEthernetCommandInterfaceEvt, &l_TIMER_ID);
}

void BSP::beginSerial()
{
  CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.begin(CC::constants::SERIAL_COMMUNICATION_INTERFACE_BAUD_RATE);
  CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.setTimeout(CC::constants::SERIAL_COMMUNICATION_INTERFACE_TIMEOUT);
  CC::AO_SerialCommandInterface->POST(&serialReadyEvt, &l_TIMER_ID);
}

void BSP::pollSerialCommand()
{
  if (CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.available() > 0)
  {
    String command = CC::constants::SERIAL_COMMUNICATION_INTERFACE_STREAM.readStringUntil('\n');
    String response = processCommandString(command);
    Serial.print(response);
  }
}

void BSP::initializeEthernet()
{
  SPI.setRX(16);
  SPI.setCS(17);
  SPI.setSCK(18);
  SPI.setTX(19);
}

void BSP::beginEthernet()
{
  uint8_t cluster_address = readClusterAddress();
  uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, cluster_address };
  // IPAddress ip(192, 168, 10, cluster_address);

  // if (Ethernet.begin(mac, ip))
  if (Ethernet.begin(mac))
  {
    CC::AO_EthernetCommandInterface->POST(&ethernetInitializedEvt, &l_TIMER_ID);
  }
  else
  {
    if (not Ethernet.hardwareStatus())
    {
      Serial.println("No Ethernet hardware detected. Check pinouts, wiring.");
    }
    else if (not Ethernet.linkStatus())
    {
      Serial.println("No Ethernet link detected. Check cable connections.");
    }
    else
    {
      Serial.println("Ethernet not initialized with mac and IP address.");
    }
  }
}

void BSP::beginEthernetServer()
{
  server.begin(CC::constants::server_port);
  CC::AO_EthernetCommandInterface->POST(&ethernetServerInitializedEvt, &l_TIMER_ID);
}

void BSP::pollEthernetCommand()
{
  WiFiClient client = server.accept();
  if (client && client.available())
  {
    String command = client.readStringUntil('\n');
    String response = processCommandString(command);
    client.write(response.c_str());
  }
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
  QF::TICK_X(0, &l_TIMER_ID); // process time events for tick rate 0
}
//............................................................................
void QF::onStartup()
{
  // configure the timer-counter channel........
  system_clock.attach_ms(CC::constants::MILLISECONDS_PER_SECOND / BSP::TICKS_PER_SEC,
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
