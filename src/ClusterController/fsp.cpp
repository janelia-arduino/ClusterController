#include "fsp.hpp"
#include "commands.hpp"


using namespace QP;

using namespace CC;

static QSpyId const l_FSP_ID = {0U}; // QSpy source ID

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
static QEvt const ethernetServerConnectedEvt = {ETHERNET_SERVER_CONNECTED_SIG, 0U, 0U};

//----------------------------------------------------------------------------
// Local functions

void FSP::ClusterController_setup()
{
  static QF_MPOOL_EL(QP::QEvt) smlPoolSto[4*constants::prism_count_max + 10];
  static QF_MPOOL_EL(CC::PrismCommandEvt) medPoolSto[4*constants::prism_count_max + 10];

  QF::init(); // initialize the framework

  QS_INIT(nullptr);

  BSP::init(); // initialize the BSP

  // initialize the event pools...
  QP::QF::poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));
  QP::QF::poolInit(medPoolSto, sizeof(medPoolSto), sizeof(medPoolSto[0]));

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

  static QEvt const *cluster_queueSto[4*constants::prism_count_max + 10];
  AO_Cluster->start(4U, // priority
    cluster_queueSto, Q_DIM(cluster_queueSto),
    (void *)0, 0U); // no stack

  //...
}

void FSP::Cluster_initializeAndSubscribe(QActive * const ao, QEvt const * e)
{
  Cluster * const cluster = static_cast<Cluster * const>(ao);
  QS_OBJ_DICTIONARY(&(cluster->cluster_time_evt_));
  QS_SIG_DICTIONARY(CLUSTER_TIMEOUT_SIG, ao);
  QS_SIG_DICTIONARY(POWER_ON_SIG, ao);
  QS_SIG_DICTIONARY(POWER_OFF_SIG, ao);
  QS_SIG_DICTIONARY(HOME_SIG, ao);
  QS_SIG_DICTIONARY(HOMED_SIG, ao);
  QS_SIG_DICTIONARY(WRITE_TARGET_POSITION_SIG, ao);

  BSP::initializeCluster();

  for (uint8_t n = 0; n < constants::prism_count_max; ++n)
  {
    cluster->prisms_[n]->init(ao->m_prio); // take the initial tran. for Prism
  }

  ao->subscribe(RESET_SIG);
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

void FSP::Cluster_armClusterTimer(QActive * const ao, QEvt const * e)
{
  Cluster * const cluster = static_cast<Cluster * const>(ao);
  cluster->cluster_time_evt_.armX(constants::ticks_per_second * constants::cluster_timer_delay_s, constants::ticks_per_second/constants::cluster_timer_frequency_hz);
}

void FSP::Cluster_disarmClusterTimer(QActive * const ao, QEvt const * e)
{
  Cluster * const cluster = static_cast<Cluster * const>(ao);
  cluster->cluster_time_evt_.disarm();
}

void FSP::Cluster_powerOnAll(QActive * const ao, QEvt const * e)
{
  BSP::powerOnAll();
  Cluster * const cluster = static_cast<Cluster * const>(ao);
  QS_BEGIN_ID(USER_COMMENT, cluster->m_prio)
    QS_STR("all prisms powered on");
  QS_END()
}

void FSP::Cluster_powerOffAll(QActive * const ao, QEvt const * e)
{
  BSP::powerOffAll();
  Cluster * const cluster = static_cast<Cluster * const>(ao);
  QS_BEGIN_ID(USER_COMMENT, cluster->m_prio)
    QS_STR("all prisms powered off");
  QS_END()
}

void FSP::Cluster_powerOnAllAndDispatch(QActive * const ao, QEvt const * e)
{
  Cluster_powerOnAll(ao, e);
  Cluster_dispatchToAll(ao, e);
}

void FSP::Cluster_dispatch(QP::QActive * const ao, QP::QEvt const * e)
{
  Cluster * const cluster = static_cast<Cluster * const>(ao);
  PrismCommandEvt const * pce = static_cast<PrismCommandEvt const *>(e);
  if ((pce->prism_address < Q_DIM(Prism::instances)) && (cluster->prisms_[pce->prism_address] != nullptr))
  {
    cluster->prisms_[pce->prism_address]->dispatch(e, ao->m_prio);
  }
}

void FSP::Cluster_dispatchToAll(QP::QActive * const ao, QP::QEvt const * e)
{
  Cluster * const cluster = static_cast<Cluster * const>(ao);
  for (uint8_t n = 0U; n < constants::prism_count_max; ++n)
  {
    if (cluster->prisms_[n] != nullptr)
    {
      cluster->prisms_[n]->dispatch(e, ao->m_prio);
    }
  }
}

void FSP::Prism_initialize(QP::QHsm * const hsm, QP::QEvt const * e)
{
  static bool dict_sent = false;
  if (!dict_sent)
  {
    dict_sent = true;

    // object dictionaries for Prism pool...
    QS_OBJ_DICTIONARY(&Prism::instances[0]);
    QS_OBJ_DICTIONARY(&Prism::instances[1]);
    QS_OBJ_DICTIONARY(&Prism::instances[2]);
    QS_OBJ_DICTIONARY(&Prism::instances[3]);
    QS_OBJ_DICTIONARY(&Prism::instances[4]);
    QS_OBJ_DICTIONARY(&Prism::instances[5]);
    QS_OBJ_DICTIONARY(&Prism::instances[6]);
  }
  // local signals
  QS_SIG_DICTIONARY(POWER_ON_SIG, hsm);

  (void)e; // unused parameter
}

void FSP::Prism_setup(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  BSP::setupPrism(prism->prism_address_);
}

bool FSP::Prism_communicating(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  return BSP::communicating(prism->prism_address_);
}

void FSP::Prism_setupParametersAndEnable(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  BSP::setupParametersAndEnable(prism->prism_address_);
}

void FSP::Prism_recordDisconnected(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  QS_BEGIN_ID(USER_COMMENT, AO_Cluster->m_prio)
    QS_STR("prism disconnected");
    QS_U8(0, prism->prism_address_);
  QS_END()
}

void FSP::Prism_recordSetupAndCommunicating(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  QS_BEGIN_ID(USER_COMMENT, AO_Cluster->m_prio)
    QS_STR("prism setup and communicating");
    QS_U8(0, prism->prism_address_);
  QS_END()
}

void FSP::Prism_beginHome(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  BSP::beginHome(prism->prism_address_);
  QS_BEGIN_ID(USER_COMMENT, AO_Cluster->m_prio)
    QS_STR("prism homing");
    QS_U8(0, prism->prism_address_);
  QS_END()
}

void FSP::Prism_endHome(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  BSP::endHome(prism->prism_address_);
}

bool FSP::Prism_homed(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  bool homed = BSP::homed(prism->prism_address_);
  if (homed)
  {
    PrismCommandEvt *pcev = Q_NEW(PrismCommandEvt, HOMED_SIG);
    pcev->prism_address = prism->prism_address_;
    AO_Cluster->POST(pcev, &l_FSP_ID);
  }
  return homed;
}

void FSP::Prism_recordHomed(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  QS_BEGIN_ID(USER_COMMENT, AO_Cluster->m_prio)
    QS_STR("prism homed");
    QS_U8(0, prism->prism_address_);
  QS_END()
}

void FSP::Prism_writeTargetPosition(QP::QHsm * const hsm, QP::QEvt const * e)
{
  Prism * const prism = static_cast<Prism * const>(hsm);
  PrismCommandEvt const * pce = static_cast<PrismCommandEvt const *>(e);
  BSP::writeTargetPosition(pce->prism_address, pce->position_mm);
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
  sci->serial_time_evt_.armX(constants::ticks_per_second * constants::serial_timer_delay_s, constants::ticks_per_second/constants::serial_timer_frequency_hz);
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
  eci->ethernet_time_evt_.armX(constants::ticks_per_second * constants::ethernet_timer_delay_s, constants::ticks_per_second/constants::ethernet_timer_frequency_hz);
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
    AO_EthernetCommandInterface->POST(&ethernetInitializedEvt, &l_FSP_ID);
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
    QS_BEGIN_ID(USER_COMMENT, eci->m_prio)
      QS_STR("string command");
    QS_END()
    QF::PUBLISH(&processStringCommandEvt, &l_FSP_ID);
  }
  else
  {
    QS_BEGIN_ID(USER_COMMENT, eci->m_prio)
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

void FSP::processStringCommand(const char * command, char * response)
{
  // strcpy(response, command);
  // if (strcmp(command, "RESET") == 0)
  // {
  //   QF::PUBLISH(&resetEvt, &l_FSP_ID);
  // }
  // if (strcmp(command, "LED_ON") == 0)
  // {
  //   BSP::ledOn();
  // }
  // else if (strcmp(command, "LED_OFF") == 0)
  // {
  //   BSP::ledOff();
  // }
  // else if (strcmp(command, "POWER_ON_ALL") == 0)
  // {
  //   AO_Cluster->POST(&powerOnEvt, &l_FSP_ID);
  // }
  // else if (strcmp(command, "POWER_OFF_ALL") == 0)
  // {
  //   AO_Cluster->POST(&powerOffEvt, &l_FSP_ID);
  // }
  // else if (strcmp(command, "RCA") == 0)
  // {
  //   uint8_t cluster_address = BSP::readClusterAddress();
  //   sprintf(response, "%d", cluster_address);
  // }
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
  // QF::PUBLISH(&commandProcessedEvt, &l_FSP_ID);
}

uint8_t FSP::processBinaryCommand(uint8_t const *command_buffer,
    size_t command_byte_count,
    uint8_t response[constants::byte_count_per_response_max])
{
  uint8_t response_byte_count = 0;
  if (command_byte_count < 2)
  {
    response[response_byte_count++] = constants::error_response;
    return response_byte_count;
  }
  uint8_t command_buffer_position = 0;
  uint8_t protocol_version = command_buffer[command_buffer_position++];
  if (protocol_version == 0x01)
  {
    uint8_t command_number = command_buffer[command_buffer_position++];
    switch (command_number)
    {
      case READ_CLUSTER_ADDRESS_CMD:
      {
        response[response_byte_count++] = BSP::readClusterAddress();
        break;
      }
      case CHECK_COMMUNICATION_CMD:
      {
        memcpy(response, &constants::check_communication_response, sizeof(constants::check_communication_response));
        response_byte_count += sizeof(constants::check_communication_response);
        break;
      }
      case RESET_CMD:
      {
        response[response_byte_count++] = command_number;
        CommandEvt *cev = Q_NEW(CommandEvt, RESET_SIG);
        QF::PUBLISH(cev, &l_FSP_ID);
        break;
      }
      case BEEP_CMD:
      {
        if (command_byte_count < 4)
        {
          response[response_byte_count++] = constants::error_response;
          return response_byte_count;
        }
        response[response_byte_count++] = command_number;
        uint16_t duration_ms;
        memcpy(&duration_ms, command_buffer + command_buffer_position, sizeof(duration_ms));
        // QS_BEGIN_ID(USER_COMMENT, AO_EthernetCommandInterface->m_prio)
        //   QS_U16(5, duration_ms_1);
        //   QS_U16(5, duration_ms);
        // QS_END()
        BSP::beep(duration_ms);
        break;
      }
      case LED_OFF_CMD:
      {
        response[response_byte_count++] = command_number;
        BSP::ledOff();
        break;
      }
      case LED_ON_CMD:
      {
        response[response_byte_count++] = command_number;
        BSP::ledOn();
        break;
      }
      case POWER_OFF_ALL_CMD:
      {
        response[response_byte_count++] = command_number;
        CommandEvt *cev = Q_NEW(CommandEvt, POWER_OFF_SIG);
        AO_Cluster->POST(cev, &l_FSP_ID);
        break;
      }
      case POWER_ON_ALL_CMD:
      {
        response[response_byte_count++] = command_number;
        CommandEvt *cev = Q_NEW(CommandEvt, POWER_ON_SIG);
        AO_Cluster->POST(cev, &l_FSP_ID);
        break;
      }
      case HOME_CMD:
      {
        response[response_byte_count++] = command_number;
        uint8_t prism_address;
        memcpy(&prism_address, command_buffer + command_buffer_position, sizeof(prism_address));

        PrismCommandEvt *pcev = Q_NEW(PrismCommandEvt, HOME_SIG);
        pcev->prism_address = prism_address;
        AO_Cluster->POST(pcev, &l_FSP_ID);
        break;
      }
      case HOME_ALL_CMD:
      {
        response[response_byte_count++] = command_number;
        for (uint8_t n = 0; n < constants::prism_count_max; ++n)
        {
          PrismCommandEvt *pcev = Q_NEW(PrismCommandEvt, HOME_SIG);
          pcev->prism_address = n;
          AO_Cluster->POST(pcev, &l_FSP_ID);
        }
        break;
      }
      case WRITE_TARGET_POSITION_CMD:
      {
        response[response_byte_count++] = command_number;
        uint8_t prism_address;
        memcpy(&prism_address, command_buffer + command_buffer_position, sizeof(prism_address));
        command_buffer_position += sizeof(prism_address);
        uint16_t position_mm;
        memcpy(&position_mm, command_buffer + command_buffer_position, sizeof(position_mm));

        PrismCommandEvt *pcev = Q_NEW(PrismCommandEvt, WRITE_TARGET_POSITION_SIG);
        pcev->prism_address = prism_address;
        pcev->position_mm = position_mm;
        AO_Cluster->POST(pcev, &l_FSP_ID);
        break;
      }
      default:
      {
        response[response_byte_count++] = constants::error_response;
        break;
      }
    }
  }
  return response_byte_count;
}
