#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/MultiArrayLayout.h"

// defined at http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html
namespace ROSMessages {
	namespace std_msgs {
		class Float32MultiArray : public FROSBaseMsg {
		public:
			Float32MultiArray() {
				_MessageType = "std_msgs/Float32MultiArray";
			}

			//# specification of data layout
			MultiArrayLayout layout;

			//# array of data
			TArray<float> data;
		};
	}
}
