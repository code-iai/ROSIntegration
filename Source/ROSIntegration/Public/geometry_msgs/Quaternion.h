#pragma once 

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace geometry_msgs {
		class Quaternion: public FROSBaseMsg {
		public:
			Quaternion() {
				_MessageType = "geometry_msgs/Quaternion";
			}

			Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w){
				_MessageType = "geometry_msgs/Quaternion";
			}

			double x;
			double y;
			double z;
			double w;

		};


	}
}
