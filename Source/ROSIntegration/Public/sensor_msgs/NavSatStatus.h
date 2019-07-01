#pragma once

#include "ROSBaseMsg.h"

// http://docs.ros.org/api/sensor_msgs/html/msg/NavSatStatus.html
namespace ROSMessages {
	namespace sensor_msgs {
		class NavSatStatus : public FROSBaseMsg {
			public:
				NavSatStatus() {
					_MessageType = "sensor_msgs/NavSatStatus";
				}

				// Status Constants
				enum Status : int8 {
					STATUS_NO_FIX = -1,	// Unable to fix position.
					STATUS_FIX = 0,		// Unaugmented fix.
					STATUS_SBAS_FIX = 1,	// With satellite-based augmentation.
					STATUS_GBAS_FIX = 2	// With ground-based augmentation.
				};

				// Whether to output an augmented fix is determined by both the fix type and the last time differential
				// corrections were received. A fix is valid when status >= STATUS_FIX.
				Status status;

				// Service Constants
				enum Service : uint16 {
					SERVICE_GPS = 1,
					SERVICE_GLONASS = 2,
					SERVICE_COMPASS = 4, // Includes BeiDou
					SERVICE_GALILEO = 8
				};

				// Bits defining which Global Navigation Satellite System signals were used by the receiver.
				Service service;
		};
	}
}
