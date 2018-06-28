#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages {
	namespace geometry_msgs {
		class Point : public FROSBaseMsg {
		public:
			Point() : Point(0, 0, 0) {}

			Point(double x, double y, double z) : x(x), y(y), z(z) {
				_MessageType = "geometry_msgs/Point";
			}

			Point(FVector v) : Point(v.X, v.Y, v.Z) {}

			double x;
			double y;
			double z;
		};
	}
}
