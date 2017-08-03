#include "ros_time.h"

namespace rosbridge2cpp {
  ROSTime ROSTime::now(){
    unsigned long seconds_since_epoch = 
      std::chrono::duration_cast<std::chrono::seconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
    unsigned long long nanoseconds_since_epoch = 
      std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
    unsigned long nanosecond_difference = nanoseconds_since_epoch - (seconds_since_epoch * 1000000000ul);
    return ROSTime(seconds_since_epoch, nanosecond_difference);
  }
} 
