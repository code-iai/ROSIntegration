#pragma once

#include "ROSBaseMsg.h"

#include "geometry_msgs/TransformStamped.h"

namespace ROSMessages{
	namespace tf2_msgs{
		class TFMessage: public FROSBaseMsg {
		public:
			TFMessage() {
				_MessageType = "tf2_msgs/TFMessage";
			}
			TArray<geometry_msgs::TransformStamped> transforms;
		};
	}
}
