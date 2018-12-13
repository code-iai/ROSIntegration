#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Float32MultiArray.h"
#include "grid_map_msgs/GridMapInfo.h"
#include "geometry_msgs/Pose.h"

namespace ROSMessages {
	namespace grid_map_msgs {
		class GridMap : public FROSBaseMsg {
		public:
			GridMap() {
				_MessageType = "grid_map_msgs/GridMap";
			}

			//# Grid map header
			//GridMapInfo info
			GridMapInfo info;

			//# Grid map layer names.
			//string[] layers
			TArray<FString> layers;

			//# Grid map basic layer names(optional).The basic layers
			//# determine which layers from `layers` need to be valid
			//# in order for a cell of the grid map to be valid.
			//string[] basic_layers
			TArray<FString> basic_layers;

			//# Grid map data.
			//std_msgs / Float32MultiArray[] data
			TArray<std_msgs::Float32MultiArray> data;

			//# Row start index(default 0).
			//uint16 outer_start_index
			uint16 outer_start_index;

			//# Column start index(default 0).
			//uint16 inner_start_index
			uint16 inner_start_index;
		};
	}
}
