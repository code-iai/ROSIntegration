#pragma once

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"
#include "NavSatStatus.h"

// http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
namespace ROSMessages {
	namespace sensor_msgs {
		class NavSatFix : public FROSBaseMsg {
			public:
				NavSatFix() {
					_MessageType = "sensor_msgs/NavSatFix";
				}

				// Standard Header
				std_msgs::Header header;

				// Satellite Fix Status Information
				NavSatStatus status;

				// Latitude [degrees]. Positive is north of equator; negative is south.
				double latitude;
				// Longitude [degrees]. Positive is east of prime meridian; negative is west.
				double longitude;
				// Altitude [m]. Positive is above the WGS 84 ellipsoid.
				double altitude;

				// Position covariance [m^2] defined relative to a tangential plane through reported position.
				TArray<double> position_covariance;

				// Position Covariance Types
				enum CovarianceType : uint8 {
					COVARIANCE_TYPE_UNKNOWN = 0,
					COVARIANCE_TYPE_APPROXIMATED = 1,
					COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
					COVARIANCE_TYPE_KNOWN = 3
				};

				// The precision of the position covariance.
				CovarianceType position_covariance_type;
		};
	}
}
