#pragma once 

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

// https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html
namespace ROSMessages {
	namespace sensor_msgs {
		class MagneticField : public FROSBaseMsg {
		public:
			MagneticField() {
				_MessageType = "sensor_msgs/MagneticField";
			}

			// Header header
			std_msgs::Header header;

			geometry_msgs::Vector3 magnetic_field;
			TArray<double> magnetic_field_covariance;

		};
	}
}
