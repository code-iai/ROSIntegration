#pragma once

#include "ROSBaseMsg.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

namespace ROSMessages {
	namespace geometry_msgs {
		class Pose : public FROSBaseMsg {
		public:
			Pose() {
				_MessageType = "geometry_msgs/Pose";
			}

			geometry_msgs::Point position;
			geometry_msgs::Quaternion orientation;
		};
	}
}
