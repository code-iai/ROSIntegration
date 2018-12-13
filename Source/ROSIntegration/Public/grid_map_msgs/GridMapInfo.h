#pragma once

#include "ROSBaseMsg.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace ROSMessages {
	namespace grid_map_msgs {
		class GridMapInfo : public FROSBaseMsg {
		public:
			GridMapInfo() {
				_MessageType = "grid_map_msgs/GridMapInfo";
			}

			//# Header(time and frame)
			//Header header
			std_msgs::Header header;

			//# Resolution of the grid[m / cell].
			//float64 resolution
			double resolution;

			//# Length in x - direction[m].
			//float64 length_x
			double length_x;

			//# Length in y - direction[m].
			//float64 length_y
			double length_y;

			//# Pose of the grid map center in the frame defined in `header`[m].
			//geometry_msgs / Pose pose
			geometry_msgs::Pose pose;
		};
	}
}
