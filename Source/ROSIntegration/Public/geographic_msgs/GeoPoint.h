#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Float32.h"

namespace ROSMessages {
	namespace geographic_msgs {
		class GeoPoint : public FROSBaseMsg {
		public:
			GeoPoint() {
				_MessageType = "geographic_msgs/GeoPoint";
			}

			double latitude;
			double longitude;
			double altitude;
		};
	}
}
