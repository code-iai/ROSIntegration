#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class Int64 : public FROSBaseMsg {
		public:
			Int64() : Int64(0.f) {}

			Int64(int64 data) {
				_MessageType = "std_msgs/Int64";
				_Data = data;
			}

		//private:
			int64 _Data;
		};
	}
}
