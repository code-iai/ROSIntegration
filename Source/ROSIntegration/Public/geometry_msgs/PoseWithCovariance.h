#pragma once

#include "ROSBaseMsg.h"

#include "geometry_msgs/Pose.h"

// http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
namespace ROSMessages {
	namespace geometry_msgs {
		class PoseWithCovariance : public FROSBaseMsg {
		public:
			PoseWithCovariance() {
				_MessageType = "geometry_msgs/PoseWithCovariance";
			}

			geometry_msgs::Pose pose;
			TArray<double> covariance;
		};
	}
}
