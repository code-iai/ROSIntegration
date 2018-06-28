#pragma once 

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"

namespace ROSMessages {
	namespace sensor_msgs {
		class Image : public FROSBaseMsg {
		public:
			Image() {
				_MessageType = "sensor_msgs/Image";
			}

			ROSMessages::std_msgs::Header header;
			uint32 height;
			uint32 width;
			FString encoding;
			uint8 is_bigendian;
			uint32 step;
			TArray<uint8> data;
			// To avoid copy operations of the image data, 
			// hand over a pointer to the uint8 data. 
			// Please note, that the memory this pointer points to must be valid until this message has been published.
			uint8* data_ptr;
		};
	}
}
