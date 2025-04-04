#include "fsp.hpp"


using namespace QP;

using namespace CC;

static QSpyId const l_FSP_ID = {0U}; // QSpy source ID

static CommandEvt const resetEvt = {RESET_SIG, 0U, 0U};
static CommandEvt const powerOnEvt = {POWER_ON_SIG, 0U, 0U};
static CommandEvt const powerOffEvt = {POWER_OFF_SIG, 0U, 0U};

static QEvt const processBinaryCommandEvt = {PROCESS_BINARY_COMMAND_SIG, 0U, 0U};
static QEvt const processStringCommandEvt = {PROCESS_STRING_COMMAND_SIG, 0U, 0U};
static QEvt const commandProcessedEvt = {COMMAND_PROCESSED_SIG, 0U, 0U};

static QEvt const activateSerialCommandInterfaceEvt = {ACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const deactivateSerialCommandInterfaceEvt = {DEACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const serialReadyEvt = {SERIAL_READY_SIG, 0U, 0U};
static QEvt const serialCommandAvailableEvt = {SERIAL_COMMAND_AVAILABLE_SIG, 0U, 0U};

static QEvt const activateEthernetCommandInterfaceEvt = {ACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const deactivateEthernetCommandInterfaceEvt = {DEACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, 0U, 0U};
static QEvt const ethernetInitializedEvt = {ETHERNET_INITIALIZED_SIG, 0U, 0U};
static QEvt const ethernetConnectedEvt = {ETHERNET_CONNECTED_SIG, 0U, 0U};
static QEvt const ethernetServerConnectedEvt = {ETHERNET_SERVER_CONNECTED_SIG, 0U, 0U};

//----------------------------------------------------------------------------
// Local functions

void FSP::ClusterController_setup()
{
  QF::init(); // initialize the framework

  QS_INIT(nullptr);

  BSP::init(); // initialize the BSP

  // object dictionaries for AOs...
  QS_OBJ_DICTIONARY(CC::AO_Cluster);
  QS_OBJ_DICTIONARY(CC::AO_SerialCommandInterface);
  QS_OBJ_DICTIONARY(CC::AO_EthernetCommandInterface);
  QS_OBJ_DICTIONARY(CC::AO_Watchdog);

  QS_OBJ_DICTIONARY(&l_FSP_ID);

  // signal dictionaries for globally published events...
  QS_SIG_DICTIONARY(CC::RESET_SIG, nullptr);
  QS_SIG_DICTIONARY(CC::POWER_ON_SIG, nullptr);
  QS_SIG_DICTIONARY(CC::POWER_OFF_SIG, nullptr);
  QS_SIG_DICTIONARY(CC::SERIAL_COMMAND_AVAILABLE_SIG, nullptr);
  QS_SIG_DICTIONARY(CC::ETHERNET_COMMAND_AVAILABLE_SIG, nullptr);
  QS_SIG_DICTIONARY(CC::COMMAND_PROCESSED_SIG, nullptr);

  // user record dictionaries
  QS_USR_DICTIONARY(ETHERNET_LOG);
  QS_USR_DICTIONARY(USER_COMMENT);

  // setup the QS filters...
  // QS_GLB_FILTER(QP::QS_SM_RECORDS); // state machine records
  // QS_GLB_FILTER(QP::QS_AO_RECORDS); // active object records
  QS_GLB_FILTER(QP::QS_UA_RECORDS); // all user records

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
  // AO_SerialCommandInterface->POST(&activateSerialCommandInterfaceEvt, &l_FSP_ID);
  AO_EthernetCommandInterface->POST(&activateEthernetCommandInterfaceEvt, &l_FSP_ID);
}

void FSP::Cluster_deactivateCommandInterfaces(QActive * const ao, QEvt const * e)
{
  // AO_SerialCommandInterface->POST(&deactivateSerialCommandInterfaceEvt, &l_FSP_ID);
  AO_EthernetCommandInterface->POST(&deactivateEthernetCommandInterfaceEvt, &l_FSP_ID);
}

void FSP::Cluster_powerOn(QActive * const ao, QEvt const * e)
{
  BSP::powerOn();
}

void FSP::Cluster_powerOff(QActive * const ao, QEvt const * e)
{
  BSP::powerOff();
}

void FSP::SerialCommandInterface_initializeAndSubscribe(QActive * const ao, QEvt const * e)
{
  ao->subscribe(SERIAL_COMMAND_AVAILABLE_SIG);
  ao->subscribe(ETHERNET_COMMAND_AVAILABLE_SIG);
  ao->subscribe(COMMAND_PROCESSED_SIG);

  SerialCommandInterface * const sci = static_cast<SerialCommandInterface * const>(ao);
  QS_OBJ_DICTIONARY(&(sci->serial_time_evt_));
  QS_SIG_DICTIONARY(SERIAL_TIMEOUT_SIG, ao);
  QS_SIG_DICTIONARY(ACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, ao);
  QS_SIG_DICTIONARY(DEACTIVATE_SERIAL_COMMAND_INTERFACE_SIG, ao);
  QS_SIG_DICTIONARY(SERIAL_READY_SIG, ao);
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
  ao->subscribe(SERIAL_COMMAND_AVAILABLE_SIG);
  ao->subscribe(ETHERNET_COMMAND_AVAILABLE_SIG);
  ao->subscribe(PROCESS_BINARY_COMMAND_SIG);
  ao->subscribe(PROCESS_STRING_COMMAND_SIG);
  ao->subscribe(COMMAND_PROCESSED_SIG);

  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  QS_OBJ_DICTIONARY(&(eci->ethernet_time_evt_));
  QS_SIG_DICTIONARY(ETHERNET_TIMEOUT_SIG, ao);
  QS_SIG_DICTIONARY(ACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, ao);
  QS_SIG_DICTIONARY(DEACTIVATE_ETHERNET_COMMAND_INTERFACE_SIG, ao);
  QS_SIG_DICTIONARY(ETHERNET_INITIALIZED_SIG, ao);
  QS_SIG_DICTIONARY(ETHERNET_SERVER_CONNECTED_SIG, ao);
}

void FSP::EthernetCommandInterface_armEthernetTimer(QActive * const ao, QEvt const * e)
{
  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  eci->ethernet_time_evt_.armX(constants::ticks_per_second, constants::ticks_per_second/100);
}

void FSP::EthernetCommandInterface_disarmEthernetTimer(QActive * const ao, QEvt const * e)
{
  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  eci->ethernet_time_evt_.disarm();
}

void FSP::EthernetCommandInterface_initializeEthernet(QActive * const ao, QEvt const * e)
{
  bool ethernet_initialized = BSP::initializeEthernet();
  if (ethernet_initialized)
  {
    QS_BEGIN_ID(USER_COMMENT, AO_EthernetCommandInterface->m_prio)
      QS_STR("ethernet initialized");
    QS_END()
    AO_EthernetCommandInterface->POST(&ethernetInitializedEvt, &l_FSP_ID);
  }
}

void FSP::EthernetCommandInterface_checkConnection(QActive * const ao, QEvt const * e)
{
  bool ethernet_connected = BSP::ethernetConnected();
  if (ethernet_connected)
  {
    QS_BEGIN_ID(USER_COMMENT, AO_EthernetCommandInterface->m_prio)
      QS_STR("ethernet connected");
    QS_END()
    AO_EthernetCommandInterface->POST(&ethernetConnectedEvt, &l_FSP_ID);
  }
}

void FSP::EthernetCommandInterface_pollEthernet(QActive * const ao, QEvt const * e)
{
  BSP::pollEthernet();
}

void FSP::EthernetCommandInterface_createServerConnection(QActive * const ao, QEvt const * e)
{
  bool server_connected = BSP::createEthernetServerConnection();
  if (server_connected)
  {
    AO_EthernetCommandInterface->POST(&ethernetServerConnectedEvt, &l_FSP_ID);
  }
}

void FSP::EthernetCommandInterface_analyzeCommand(QActive * const ao, QEvt const * e)
{
  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  EthernetCommandEvt const * ece = static_cast<EthernetCommandEvt const *>(e);
  eci->connection_ = ece->connection;
  eci->binary_command_ = ece->binary_command;
  eci->binary_command_byte_count_ = ece->binary_command_byte_count;

  uint8_t first_command_byte = (uint8_t)(eci->binary_command_[0]);
  if (first_command_byte > constants::first_command_byte_max_value_binary)
  {
    QS_BEGIN_ID(USER_COMMENT, AO_EthernetCommandInterface->m_prio)
      QS_STR("string command");
    QS_END()
    QF::PUBLISH(&processStringCommandEvt, &l_FSP_ID);
  }
  else
  {
    QS_BEGIN_ID(USER_COMMENT, AO_EthernetCommandInterface->m_prio)
      QS_STR("binary command");
    QS_END()
    QF::PUBLISH(&processBinaryCommandEvt, &l_FSP_ID);
  }
}

void FSP::EthernetCommandInterface_processBinaryCommand(QActive * const ao, QEvt const * e)
{
  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  eci->binary_response_byte_count_ = FSP::processBinaryCommand(eci->binary_command_,
    eci->binary_command_byte_count_,
    eci->binary_response_);
  QF::PUBLISH(&commandProcessedEvt, &l_FSP_ID);
}

void FSP::EthernetCommandInterface_writeBinaryResponse(QActive * const ao, QEvt const * e)
{
  EthernetCommandInterface * const eci = static_cast<EthernetCommandInterface * const>(ao);
  BSP::writeEthernetBinaryResponse(eci->connection_, eci->binary_response_, eci->binary_response_byte_count_);
}

void FSP::Watchdog_initializeAndSubscribe(QActive * const ao, QEvt const * e)
{
  ao->subscribe(RESET_SIG);
  BSP::initializeWatchdog();
}

void FSP::Watchdog_armWatchdogTimer(QActive * const ao, QEvt const * e)
{
  Watchdog * const watchdog = static_cast<Watchdog * const>(ao);
  watchdog->watchdog_time_evt_.armX(constants::ticks_per_second, constants::ticks_per_second);

  QS_OBJ_DICTIONARY(&(watchdog->watchdog_time_evt_));
  QS_SIG_DICTIONARY(WATCHDOG_TIMEOUT_SIG, ao);
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

uint8_t FSP::processBinaryCommand(uint8_t const * command_buffer,
    size_t command_byte_count,
    uint8_t response[constants::byte_count_per_response_max])
{
  uint8_t response_byte_count = 0;
  // uint8_t second_command_byte = (uint8_t)(command_buffer[1]);
  // response[response_byte_count++] = 2;
  // response[response_byte_count++] = 0;
  // response[response_byte_count++] = second_command_byte;
  // switch (second_command_byte)
  // {
  //   case 0x01:
  //   {
  //     AO_Watchdog->POST(&resetEvt, &l_FSP_ID);
  //     appendMessage(response, response_byte_count, "Reset Command Sent to FPGA");
  //     break;
  //   }
  //   case 0x30:
  //   {
  //     AO_Arena->POST(&allOffEvt, &l_FSP_ID);
  //     appendMessage(response, response_byte_count, "Display has been stopped");
  //     break;
  //   }
  //   case 0x00:
  //   {
  //     AO_Arena->POST(&allOffEvt, &l_FSP_ID);
  //     appendMessage(response, response_byte_count, "All-Off Received");
  //     break;
  //   }
  //   case 0xFF:
  //   {
  //     AO_Arena->POST(&allOnEvt, &l_FSP_ID);
  //     appendMessage(response, response_byte_count, "All-On Received");
  //     break;
  //   }
  //   default:
  //     break;
  // }
  return response_byte_count;
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
