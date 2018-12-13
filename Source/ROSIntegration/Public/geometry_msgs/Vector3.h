#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace geometry_msgs {
		class Vector3: public FROSBaseMsg {
		public:
			Vector3() : Vector3(0, 0, 0) {}

			Vector3(double x, double y, double z) : x(x), y(y), z(z) {
				_MessageType = "geometry_msgs/Vector3";
			}

			Vector3(FVector v) : Vector3(v.X, v.Y, v.Z) {}

			double x;
			double y;
			double z;
		};
	}
}
