#pragma once

#include "ROSBaseMsg.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace ROSMessages{
	namespace geometry_msgs {
		class Transform: public FROSBaseMsg {
		public:
			Transform() {
				_MessageType = "geometry_msgs/Transform";
			}
			geometry_msgs::Vector3 translation;
			geometry_msgs::Quaternion rotation;
		};
	}
}
