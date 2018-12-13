#pragma once

#include "ROSBaseMsg.h"

#include "geometry_msgs/Pose.h"
#include "std_msgs/Header.h"

namespace ROSMessages {
	namespace geometry_msgs {
		class PoseStamped : public FROSBaseMsg {
		public:
			PoseStamped() {
				_MessageType = "geometry_msgs/PoseStamped";
			}

			std_msgs::Header header;
			geometry_msgs::Pose pose;
		};
	}
}
