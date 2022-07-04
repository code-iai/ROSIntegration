#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class Int32 : public FROSBaseMsg {
		public:
			Int32() : Int32(0.f) {}

			Int32(float data) {
				_MessageType = "std_msgs/Int32";
				_Data = data;
			}

		//private:
			int32 _Data;
		};
	}
}
