#pragma once

#include "ROSBaseMsg.h"

#include "geometry_msgs/Vector3.h"

namespace ROSMessages {
	namespace geometry_msgs {
		class Twist : public FROSBaseMsg {
		public:
			Twist() {
				_MessageType = "geometry_msgs/Twist";
			}

			geometry_msgs::Vector3 linear;
			geometry_msgs::Vector3 angular;
		};
	}
}
