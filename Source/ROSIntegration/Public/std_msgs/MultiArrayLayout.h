#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/MultiArrayDimension.h"

// defined at http://docs.ros.org/api/std_msgs/html/msg/MultiArrayLayout.html
namespace ROSMessages {
	namespace std_msgs {
		class MultiArrayLayout : public FROSBaseMsg {
		public:
			MultiArrayLayout() {
				_MessageType = "std_msgs/MultiArrayLayout";
			}

			//# Array of dimension properties
			TArray<MultiArrayDimension> dim;

			//# padding elements at front of data
			uint32 data_offset;
		};
	}
}
