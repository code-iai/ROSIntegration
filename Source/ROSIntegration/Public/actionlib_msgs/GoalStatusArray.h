#pragma once

#include "ROSBaseMsg.h"
#include "GoalStatus.h"
#include "std_msgs/Header.h"

namespace ROSMessages {
	namespace actionlib_msgs {
		class GoalStatusArray : public FROSBaseMsg {
		public:
			GoalStatusArray() {
				_MessageType = "actionlib_msgs/GoalStatusArray";
			}

			// # Stores the statuses for goals that are currently being tracked
			// # by an action server
			std_msgs::Header header;
			TArray<GoalStatus> status_list;
		};
	}
}
