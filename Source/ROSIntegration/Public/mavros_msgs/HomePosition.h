#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Header.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"


namespace ROSMessages {
	namespace mavros_msgs {
		class HomePosition : public FROSBaseMsg {
		public:
			HomePosition() {
				_MessageType = "mavros_msgs/HomePosition";
			}

			std_msgs::Header header;
			geographic_msgs::GeoPoint geo;
			geometry_msgs::Point position;
			geometry_msgs::Quaternion orientation;
			geometry_msgs::Vector3 approach;
		};
	}
}
