#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Transform.h"

namespace ROSMessages{
	namespace geometry_msgs {
		class TransformStamped: public FROSBaseMsg {
		public:
			TransformStamped() {
				_MessageType = "geometry_msgs/TransformStamped";
			}

			std_msgs::Header header;
			FString child_frame_id;
			geometry_msgs::Transform transform;
		};
	}
}
