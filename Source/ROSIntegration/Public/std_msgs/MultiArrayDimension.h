#pragma once

#include "ROSBaseMsg.h"

// defined at http://docs.ros.org/api/std_msgs/html/msg/MultiArrayDimension.html
namespace ROSMessages {
	namespace std_msgs {
		class MultiArrayDimension : public FROSBaseMsg {
		public:
			MultiArrayDimension() {
				_MessageType = "std_msgs/MultiArrayDimension";
			}

			//string label   # label of given dimension
			FString label;

			//uint32 size	# size of given dimension(in type units)
			uint32 size;

			//uint32 stride  # stride of given dimension
			uint32 stride;
		};
	}
}
