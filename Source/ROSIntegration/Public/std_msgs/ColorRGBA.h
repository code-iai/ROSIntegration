#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class ColorRGBA : public FROSBaseMsg {
		public:
			ColorRGBA() : ColorRGBA(0.f, 0.f, 0.f)  {}

			ColorRGBA(float InR, float InG, float InB, float InA = 1.) {
				_MessageType = "std_msgs/ColorRGBA";
				r = InR;
				g = InG;
				b = InB;
				a = InA;
			}

			float r;
			float g;
			float b;
			float a;
		};
	}
}
