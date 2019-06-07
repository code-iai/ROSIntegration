#pragma once 

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

// http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
namespace ROSMessages {
	namespace sensor_msgs {
		class Imu : public FROSBaseMsg {
		public:
			Imu() {
				_MessageType = "sensor_msgs/Imu";
			}

			// Header header
			std_msgs::Header header;

			geometry_msgs::Quaternion orientation;
			TArray<double> orientation_covariance;

			geometry_msgs::Vector3 angular_velocity;
			TArray<double> angular_velocity_covariance;

			geometry_msgs::Vector3 linear_acceleration;
			TArray<double> linear_acceleration_covariance;
		};
	}
}
