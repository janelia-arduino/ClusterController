#ifndef FSP_HPP
#define FSP_HPP

#include <Arduino.h>

#include "constants.hpp"
#include "bsp.hpp"
#include "signals.hpp"
#include "records.hpp"

#include "ClusterController.hpp"
#include "Cluster.hpp"
#include "Prism.hpp"
#include "SerialCommandInterface.hpp"
#include "EthernetCommandInterface.hpp"
#include "Watchdog.hpp"



struct FSP
{
  static void ClusterController_setup();

  static void Cluster_initializeAndSubscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_activateCommandInterfaces(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_deactivateCommandInterfaces(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_armClusterTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_disarmClusterTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_powerOnAll(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_powerOffAll(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_powerOnAllAndDispatch(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_dispatch(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_dispatchToAll(QP::QActive * const ao, QP::QEvt const * e);

  static void Prism_initialize(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_setup(QP::QHsm * const hsm, QP::QEvt const * e);
  static bool Prism_communicating(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_setupParametersAndEnable(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_recordDisconnected(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_recordSetupAndCommunicating(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_beginHome(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_endHome(QP::QHsm * const hsm, QP::QEvt const * e);
  static bool Prism_homed(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_recordHomed(QP::QHsm * const hsm, QP::QEvt const * e);
  static void Prism_writeTargetPosition(QP::QHsm * const hsm, QP::QEvt const * e);

  static void SerialCommandInterface_initializeAndSubscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_armSerialTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_disarmSerialTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_beginSerial(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_pollSerialCommand(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_readFirstByte(QP::QActive * const ao, QP::QEvt const * e);
  static bool SerialCommandInterface_ifBinaryCommand(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_readSerialStringCommand(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_processStringCommand(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_writeSerialStringResponse(QP::QActive * const ao, QP::QEvt const * e);
  // static void SerialCommandInterface_writeSerialBinaryResponse(QP::QActive * const ao, QP::QEvt const * e);

  static void EthernetCommandInterface_initializeAndSubscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_armEthernetTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_disarmEthernetTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_initializeEthernet(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_pollEthernet(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_createServerConnection(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_analyzeCommand(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_processBinaryCommand(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_writeBinaryResponse(QP::QActive * const ao, QP::QEvt const * e);

  static void Watchdog_initializeAndSubscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void Watchdog_armWatchdogTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void Watchdog_disarmWatchdogTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void Watchdog_feedWatchdog(QP::QActive * const ao, QP::QEvt const * e);

  static void processStringCommand(const char * command, char * response);
  static uint8_t processBinaryCommand(uint8_t const * command_buffer,
    size_t command_byte_count,
    uint8_t response[CC::constants::byte_count_per_response_max]);
};

#endif // FSP_HPP
