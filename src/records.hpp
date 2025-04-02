#ifndef RECORDS_HPP
#define RECORDS_HPP

#include "qpcpp.hpp"


namespace CC
{

enum ArenaControllerRecords
{
  ETHERNET_LOG = QP::QS_USER,
  USER_COMMENT = QP::QS_USER+5,
};

} // namespace CC

#endif // RECORDS_HPP
