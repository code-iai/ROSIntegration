#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages {
	namespace geometry_msgs {
		class Point : public FROSBaseMsg {
		public:
			Point() {
				_MessageType = "geometry_msgs/Point";
			}

			Point(double x, double y, double z) : x(x), y(y), z(z) {
				_MessageType = "geometry_msgs/Point";
			}

			Point(FVector v) : x(v.X), y(v.Y), z(v.Z) {
				_MessageType = "geometry_msgs/Point";
			}

			double x;
			double y;
			double z;
		};
	}
}