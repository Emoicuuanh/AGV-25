#ifndef AGV_IO_H_
#define AGV_IO_H_
#include "stdlib.h"

void loginfoNumber(char* start_msg, int num, char* stop_msg)
{
  if (!rosSerialConnected)
    return;
  char log_msg[20];
  char result[8];
  dtostrf(num, 0, 0, result);
  sprintf(log_msg, "%s%s%s", start_msg, result, stop_msg);
  nh.loginfo(log_msg);
}

#endif