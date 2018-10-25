#pragma once

#include "ROSBaseMsg.h"
#include "ROSTime.h"

namespace ROSMessages {
	namespace actionlib_msgs {
		class GoalID : public FROSBaseMsg {
		public:
			GoalID() {
				_MessageType = "actionlib_msgs/GoalID";
			}

			// # The stamp should store the time at which this goal was requested.
			// # It is used by an action server when it tries to preempt all
			// # goals that were requested before a certain time
			FROSTime stamp;

			// # The id provides a way to associate feedback and
			// # result message with specific goal requests. The id
			// # specified must be unique.
			FString id;
		};
	}
}
