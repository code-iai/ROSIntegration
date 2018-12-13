#pragma once

#include "ROSBaseMsg.h"
#include "ROSTime.h"

namespace ROSMessages {
	namespace rosgraph_msgs {
		class Clock : public FROSBaseMsg {
		public:
			Clock() : Clock(FROSTime()) {}

			Clock(FROSTime clock) {
				_MessageType = "rosgraph_msgs/Clock";
				_Clock = clock;
			}

			//private:
			FROSTime _Clock;
		};
	}
}
