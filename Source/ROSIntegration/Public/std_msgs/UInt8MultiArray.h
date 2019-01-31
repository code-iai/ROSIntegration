#pragma once 

#include "ROSBaseMsg.h"

#include "std_msgs/MultiArrayLayout.h"

// defined at http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html
namespace ROSMessages {
	namespace std_msgs {
		class UInt8MultiArray : public FROSBaseMsg {
		public:
			UInt8MultiArray() {
				_MessageType = "std_msgs/UInt8MultiArray";
			}

			//# specification of data layout
			MultiArrayLayout layout;

			// To avoid copy operations of the image data, 
			// hand over a pointer to the uint8 data. 
			// Please note, that the memory this pointer points to must be valid until this message has been published.
			const uint8* data;
		};
	}
}
