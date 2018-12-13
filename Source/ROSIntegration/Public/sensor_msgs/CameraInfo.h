#pragma once

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"
#include "RegionOfInterest.h"


namespace ROSMessages{
	namespace sensor_msgs {
		class CameraInfo: public FROSBaseMsg {
		public:
			CameraInfo() {
				_MessageType = "sensor_msgs/CameraInfo";
				D.Init(0,5); // The size of D is undefined in the camera info as it may need changes. We'll start with 5.
				K.Init(0,9);
				R.Init(0,9);
				P.Init(0,12);
			}

			ROSMessages::std_msgs::Header header;
			uint32 height;
			uint32 width;
			FString distortion_model;
			TArray<double> D;
			TArray<double> K;
			TArray<double> R;
			TArray<double> P;
			uint32 binning_x;
			uint32 binning_y;
			RegionOfInterest roi;
		};
	}
}
