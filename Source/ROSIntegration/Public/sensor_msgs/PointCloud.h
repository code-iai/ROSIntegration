#pragma once 

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"

namespace ROSMessages {
	namespace sensor_msgs {
		struct  ChannelFloat32
		{
			FString name;
			TArray<float> valueData;
			float *valueData_ptr;
		};

		struct Point32
		{
			float x;
			float y;
			float z;
		};

		class PointCloud : public FROSBaseMsg {
		public:
			PointCloud() {
				_MessageType = "sensor_msgs/PointCloud";
			}

			ROSMessages::std_msgs::Header header;
			TArray<Point32> pointData;
			// To avoid copy operations of the image data, 
			// hand over a pointer to the float data. 
			// Please note, that the memory this pointer points to must be valid until this message has been published.
			float *pointData_ptr;

			ChannelFloat32 channel;
		};
	}
}
