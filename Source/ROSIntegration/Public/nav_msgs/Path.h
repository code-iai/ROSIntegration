#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"

namespace ROSMessages {
	namespace nav_msgs {
		class Path : public FROSBaseMsg {
		public:
			Path() {
				_MessageType = "nav_msgs/Path";
			}

			// Header header
			std_msgs::Header header;
			// geometry_msgs / PoseStamped[] poses
			TArray<geometry_msgs::PoseStamped> poses;
		};
	}
}
