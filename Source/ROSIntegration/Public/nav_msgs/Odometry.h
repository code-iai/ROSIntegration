#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"

namespace ROSMessages {
	namespace nav_msgs {
		class Odometry : public FROSBaseMsg {
		public:
			Odometry() {
				_MessageType = "nav_msgs/Odometry";
			}

			// Header header
			std_msgs::Header header;
			// string child_frame_id
			FString child_frame_id;
			// geometry_msgs / PoseWithCovariance pose
			geometry_msgs::PoseWithCovariance pose;
			// geometry_msgs / TwistWithCovariance twist
			geometry_msgs::TwistWithCovariance twist;
		};
	}
}
