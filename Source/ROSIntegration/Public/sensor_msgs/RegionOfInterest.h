#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace sensor_msgs {
		class RegionOfInterest: public FROSBaseMsg {
		public:
			RegionOfInterest() {
				_MessageType = "sensor_msgs/RegionOfInterest";
			}

			uint32 x_offset;
			uint32 y_offset;
			uint32 height;
			uint32 width;
			bool do_rectify;
		};
	}
}
