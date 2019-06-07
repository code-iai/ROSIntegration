//
//  MapMetaData.h
//  HFS
//
//  Created by Timothy Saucer on 2/19/19.
//  Copyright © 2019 Epic Games, Inc. All rights reserved.
//

#pragma once

#include "ROSBaseMsg.h"
#include "geometry_msgs/Pose.h"

namespace ROSMessages {
	namespace nav_msgs {
		class MapMetaData : public FROSBaseMsg {
		public:
			MapMetaData() {
				_MessageType = "nav_msgs/MapMetaData";
			}

			MapMetaData(FROSTime map_load_time, float resolution, uint32 width, uint32 height, geometry_msgs::Pose origin) : map_load_time(map_load_time), resolution(resolution), width(width), height(height), origin(origin)
			{
				_MessageType = "nav_msgs/MapMetaData";
			}

			// time map_load_time
			FROSTime map_load_time;

			//float32 resolution
			float resolution;

			//uint32 width
			uint32 width;

			//uint32 height
			uint32 height;

			//geometry_msgs/Pose origin
			geometry_msgs::Pose origin;
		};
	}
}


//time map_load_time
//float32 resolution
//uint32 width
//uint32 height
//geometry_msgs/Pose origin
