#include "fsp.hpp"


using namespace QP;

using namespace CC;

static QSpyId const l_FSP_ID = { 0U }; // QSpy source ID

static CommandEvt const resetEvt = {RESET_SIG, 0U, 0U};
static CommandEvt const powerOnEvt = {POWER_ON_SIG, 0U, 0U};
static CommandEvt const powerOffEvt = {POWER_OFF_SIG, 0U, 0U};

static QEvt const activateSerialCommandInterfaceEvt = {ACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const deactivateSerialCommandInterfaceEvt = {DEACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const serialReadyEvt = {SERIAL_READY_SIG, 0U, 0U};
static QEvt const serialCommandAvailableEvt = {SERIAL_COMMAND_AVAILABLE_SIG, 0U, 0U};

static QEvt const activateEthernetCommandInterfaceEvt = {ACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const deactivateEthernetCommandInterfaceEvt = {DEACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const ethernetInitializedEvt = {ETHERNET_INITIALIZED_SIG, 0U, 0U};
// static QEvt const ethernetIPAddressFoundEvt = {ETHERNET_IP_ADDRESS_FOUND_SIG, 0U, 0U};
static QEvt const ethernetServerInitializedEvt = {ETHERNET_SERVER_INITIALIZED_SIG, 0U, 0U};
static QEvt const ethernetCommandAvailableEvt = {ETHERNET_COMMAND_AVAILABLE_SIG, 0U, 0U};

static QEvt const commandProcessedEvt = {COMMAND_PROCESSED_SIG, 0U, 0U};

//----------------------------------------------------------------------------
// Local functions

void FSP::ClusterController_setup()
{
  QF::init(); // initialize the framework
  BSP::init(); // initialize the BSP

  // init publish-subscribe
  static QSubscrList subscrSto[MAX_PUB_SIG];
  QF::psInit(subscrSto, Q_DIM(subscrSto));

  // statically allocate event queues for the AOs and start them...
  static QEvt const *watchdog_queueSto[2];
  AO_Watchdog->start(1U, // priority
    watchdog_queueSto, Q_DIM(watchdog_queueSto),
    (void *)0, 0U); // no stack

  static QEvt const *serial_command_interface_queueSto[10];
  AO_SerialCommandInterface->start(2U, // priority
    serial_command_interface_queueSto, Q_DIM(serial_command_interface_queueSto),
    (void *)0, 0U); // no stack

  static QEvt const *ethernet_command_interface_queueSto[10];
  AO_EthernetCommandInterface->start(3U, // priority
    ethernet_command_interface_queueSto, Q_DIM(ethernet_command_interface_queueSto),
    (void *)0, 0U); // no stack

  static QEvt const *cluster_queueSto[10];
  AO_Cluster->start(4U, // priority
    cluster_queueSto, Q_DIM(cluster_queueSto),
    (void *)0, 0U); // no stack

  //...
}

void FSP::Cluster_initializeAndSubscribe(QActive * const ao, QEvt const * e)
{
  BSP::initializeCluster();
  ao->subscribe(RESET_SIG);
  ao->subscribe(POWER_ON_SIG);
  ao->subscribe(POWER_OFF_SIG);
}

void FSP::Cluster_activateCommandInterfaces(QActive * const ao, QEvt const * e)
{
  AO_SerialCommandInterface->POST(&activateSerialCommandInterfaceEvt, &l_FSP_ID);
  // AO_EthernetCommandInterface->POST(&activateEthernetCommandInterfaceEvt, &l_FSP_ID);
}

void FSP::Cluster_deactivateCommandInterfaces(QActive * const ao, QEvt const * e)
{
  AO_SerialCommandInterface->POST(&deactivateSerialCommandInterfaceEvt, &l_FSP_ID);
  // AO_EthernetCommandInterface->POST(&deactivateEthernetCommandInterfaceEvt, &l_FSP_ID);
}

void FSP::Cluster_powerOn(QActive * const ao, QEvt const * e)
{
  BSP::powerOn();
}

void FSP::Cluster_powerOff(QActive * const ao, QEvt const * e)
{
  BSP::powerOff();
}

void FSP::SerialCommandInterface_subscribe(QActive * const ao, QEvt const * e)
{
  ao->subscribe(SERIAL_COMMAND_AVAILABLE_SIG);
  // ao->subscribe(ETHERNET_COMMAND_AVAILABLE_SIG);
  ao->subscribe(COMMAND_PROCESSED_SIG);
}

void FSP::SerialCommandInterface_armSerialTimer(QActive * const ao, QEvt const * e)
{
  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  sci->serial_time_evt_.armX(constants::ticks_per_second/2, constants::ticks_per_second/50);
}

void FSP::SerialCommandInterface_disarmSerialTimer(QActive * const ao, QEvt const * e)
{
  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  sci->serial_time_evt_.disarm();
}

void FSP::SerialCommandInterface_beginSerial(QActive * const ao, QEvt const * e)
{
  bool serial_ready = BSP::beginSerial();
  if (serial_ready)
  {
    CC::AO_SerialCommandInterface->POST(&serialReadyEvt, &l_FSP_ID);
  }
}

void FSP::SerialCommandInterface_pollSerialCommand(QActive * const ao, QEvt const * e)
{
  bool bytes_available = BSP::pollSerialCommand();
  if (bytes_available)
  {
    QF::PUBLISH(&serialCommandAvailableEvt, &l_FSP_ID);
  }
}

void FSP::SerialCommandInterface_readFirstByte(QActive * const ao, QEvt const * e)
{
  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  sci->first_command_byte_ = BSP::readSerialByte();
}

bool FSP::SerialCommandInterface_ifBinaryCommand(QActive * const ao, QEvt const * e)
{
  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  return (sci->first_command_byte_ <= constants::first_command_byte_max_value_binary);
}

void FSP::SerialCommandInterface_readSerialStringCommand(QActive * const ao, QEvt const * e)
{
  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  BSP::readSerialStringCommand(sci->string_command_, (char)sci->first_command_byte_);
}

void FSP::SerialCommandInterface_processStringCommand(QActive * const ao, QEvt const * e)
{
  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  FSP::processStringCommand(sci->string_command_, sci->string_response_);
}

void FSP::SerialCommandInterface_writeSerialStringResponse(QActive * const ao, QEvt const * e)
{
  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  BSP::writeSerialStringResponse(sci->string_response_);
}

// void FSP::SerialCommandInterface_writeSerialBinaryResponse(QActive * const ao, QEvt const * e)
// {
// }

void FSP::EthernetCommandInterface_initializeAndSubscribe(QActive * const ao, QEvt const * e)
{
  BSP::initializeEthernet();
  // ao->subscribe(SERIAL_COMMAND_AVAILABLE_SIG);
  // ao->subscribe(ETHERNET_COMMAND_AVAILABLE_SIG);
  // ao->subscribe(COMMAND_PROCESSED_SIG);
}

void FSP::EthernetCommandInterface_armEthernetTimer(QActive * const ao, QEvt const * e)
{
  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  eci->ethernet_time_evt_.armX(constants::ticks_per_second/2, constants::ticks_per_second/50);
}

void FSP::EthernetCommandInterface_disarmEthernetTimer(QActive * const ao, QEvt const * e)
{
  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  eci->ethernet_time_evt_.disarm();
}

void FSP::EthernetCommandInterface_beginEthernet(QActive * const ao, QEvt const * e)
{
  bool ethernet_begun = BSP::beginEthernet();
  if (ethernet_begun)
  {
    CC::AO_EthernetCommandInterface->POST(&ethernetInitializedEvt, &l_FSP_ID);
  }
}

// void FSP::EthernetCommandInterface_checkForIPAddress(QActive * const ao, QEvt const * e)
// {
//   bool ip_address_found = BSP::checkForEthernetIPAddress();
//   if (ip_address_found)
//   {
//     AO_EthernetCommandInterface->POST(&ethernetIPAddressFoundEvt, &l_FSP_ID);
//   }
// }

void FSP::EthernetCommandInterface_beginServer(QActive * const ao, QEvt const * e)
{
  bool ethernet_server_begun = BSP::beginEthernetServer();
  if (ethernet_server_begun)
  {
    AO_EthernetCommandInterface->POST(&ethernetServerInitializedEvt, &l_FSP_ID);
  }
}

void FSP::EthernetCommandInterface_pollEthernetCommand(QActive * const ao, QEvt const * e)
{
  bool bytes_available = BSP::pollEthernetCommand();
  if (bytes_available)
  {
    QF::PUBLISH(&ethernetCommandAvailableEvt, &l_FSP_ID);
  }
}

// void FSP::EthernetCommandInterface_readEthernetBinaryCommand(QActive * const ao, QEvt const * e)
// {
//   EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
//   BSP::readEthernetBinaryCommand();
//   // eci->binary_command_ = BSP::readEthernetBinaryCommand(sci->first_command_byte_);
// }

// void FSP::EthernetCommandInterface_writeEthernetBinaryResponse(QActive * const ao, QEvt const * e)
// {
//   EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
//   // eci->string_command_ = BSP::readSerialStringCommand(sci->first_command_byte_);
// }

void FSP::Watchdog_initializeAndSubscribe(QActive * const ao, QEvt const * e)
{
  ao->subscribe(RESET_SIG);
  BSP::initializeWatchdog();
}

void FSP::Watchdog_armWatchdogTimer(QActive * const ao, QEvt const * e)
{
  Serial.println("arming watchdog timer");
  Watchdog * const watchdog = static_cast<Watchdog * const>(ao);
  watchdog->watchdog_time_evt_.armX(constants::ticks_per_second, constants::ticks_per_second);
}

void FSP::Watchdog_disarmWatchdogTimer(QActive * const ao, QEvt const * e)
{
  Watchdog * const watchdog = static_cast<Watchdog * const>(ao);
  watchdog->watchdog_time_evt_.disarm();
}

void FSP::Watchdog_feedWatchdog(QActive * const ao, QEvt const * e)
{
  BSP::feedWatchdog();
}

void FSP::processStringCommand(const char * command, char * response)
{
  strcpy(response, command);
  if (strcmp(command, "RESET") == 0)
  {
    QF::PUBLISH(&resetEvt, &l_FSP_ID);
  }
  if (strcmp(command, "LED_ON") == 0)
  {
    BSP::ledOn();
  }
  else if (strcmp(command, "LED_OFF") == 0)
  {
    BSP::ledOff();
  }
  else if (strcmp(command, "POWER_ON") == 0)
  {
    QF::PUBLISH(&powerOnEvt, &l_FSP_ID);
  }
  else if (strcmp(command, "POWER_OFF") == 0)
  {
    QF::PUBLISH(&powerOffEvt, &l_FSP_ID);
  }
  else if (strcmp(command, "RCA") == 0)
  {
    uint8_t cluster_address = BSP::readClusterAddress();
    sprintf(response, "%d", cluster_address);
  }
  // else if (strcmp(command, "EHS") == 0)
  // {
  //   BSP::getEthernetHardwareStatusString(response);
  // }
  // else if (strcmp(command, "ELS") == 0)
  // {
  //   BSP::getEthernetLinkStatusString(response);
  // }
  // else if (strcmp(command, "SIP") == 0)
  // {
  //   BSP::getServerIpAddressString(response);
  // }
  QF::PUBLISH(&commandProcessedEvt, &l_FSP_ID);
}
