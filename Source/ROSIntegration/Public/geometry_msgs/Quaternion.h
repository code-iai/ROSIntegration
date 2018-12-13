#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace geometry_msgs {
		class Quaternion: public FROSBaseMsg {
		public:
			Quaternion() : Quaternion(0, 0, 0, 0) {}

			Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w){
				_MessageType = "geometry_msgs/Quaternion";
			}

			Quaternion(FQuat q) : Quaternion(q.X, q.Y, q.Z, q.W) {}

			double x;
			double y;
			double z;
			double w;
		};
	}
}
