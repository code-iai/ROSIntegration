#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

namespace ROSMessages {
	namespace nav_msgs {
		class OccupancyGrid : public FROSBaseMsg {
		public:
			OccupancyGrid() {
				_MessageType = "nav_msgs/OccupancyGrid";
			}

			// Header header
			std_msgs::Header header;

			// nav_msgs/MapMetaData info
			MapMetaData info;

			// int8[] data
			// Note: BSON will coerce the int32 to int8. int8 not implemented in BSON.
			TArray<int32> data;
		};
	}
}
