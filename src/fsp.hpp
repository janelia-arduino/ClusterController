#ifndef FSP_HPP
#define FSP_HPP

#include <Arduino.h>

#include "constants.hpp"
#include "bsp.hpp"
#include "signals.hpp"

#include "ClusterController.hpp"
#include "Cluster.hpp"
#include "SerialCommandInterface.hpp"
#include "EthernetCommandInterface.hpp"
#include "Watchdog.hpp"



struct FSP
{
  static void ClusterController_setup();

  static void Cluster_initializeAndSubscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_activateCommandInterfaces(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_deactivateCommandInterfaces(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_powerOn(QP::QActive * const ao, QP::QEvt const * e);
  static void Cluster_powerOff(QP::QActive * const ao, QP::QEvt const * e);

  static void SerialCommandInterface_subscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_armSerialTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_disarmSerialTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_beginSerial(QP::QActive * const ao, QP::QEvt const * e);
  static void SerialCommandInterface_pollSerialCommand(QP::QActive * const ao, QP::QEvt const * e);
  // static void SerialCommandInterface_readFirstByte(QP::QActive * const ao, QP::QEvt const * e);
  // static bool SerialCommandInterface_ifBinaryCommand(QP::QActive * const ao, QP::QEvt const * e);
  // static void SerialCommandInterface_readSerialStringCommand(QP::QActive * const ao, QP::QEvt const * e);
  // static void SerialCommandInterface_processStringCommand(QP::QActive * const ao, QP::QEvt const * e);
  // static void SerialCommandInterface_writeSerialStringResponse(QP::QActive * const ao, QP::QEvt const * e);
  // static void SerialCommandInterface_writeSerialBinaryResponse(QP::QActive * const ao, QP::QEvt const * e);

  static void EthernetCommandInterface_initializeAndSubscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_armEthernetTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_disarmEthernetTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_beginEthernet(QP::QActive * const ao, QP::QEvt const * e);
  // static void EthernetCommandInterface_checkForIPAddress(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_beginServer(QP::QActive * const ao, QP::QEvt const * e);
  // static void EthernetCommandInterface_checkForClient(QP::QActive * const ao, QP::QEvt const * e);
  static void EthernetCommandInterface_pollEthernetCommand(QP::QActive * const ao, QP::QEvt const * e);
  // static void EthernetCommandInterface_readEthernetBinaryCommand(QP::QActive * const ao, QP::QEvt const * e);
  // static void EthernetCommandInterface_writeEthernetBinaryResponse(QP::QActive * const ao, QP::QEvt const * e);

  static void Watchdog_initializeAndSubscribe(QP::QActive * const ao, QP::QEvt const * e);
  static void Watchdog_armWatchdogTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void Watchdog_disarmWatchdogTimer(QP::QActive * const ao, QP::QEvt const * e);
  static void Watchdog_feedWatchdog(QP::QActive * const ao, QP::QEvt const * e);

  static void processStringCommand(const char * command, char * response);
};

#endif // FSP_HPP
