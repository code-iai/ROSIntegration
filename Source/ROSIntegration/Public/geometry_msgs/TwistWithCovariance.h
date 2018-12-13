#pragma once

#include "ROSBaseMsg.h"

#include "geometry_msgs/Twist.h"

// http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovariance.html
namespace ROSMessages {
	namespace geometry_msgs {
		class TwistWithCovariance : public FROSBaseMsg {
		public:
			TwistWithCovariance() {
				_MessageType = "geometry_msgs/TwistWithCovariance";
			}

			geometry_msgs::Twist twist;
			TArray<double> covariance;
		};
	}
}
