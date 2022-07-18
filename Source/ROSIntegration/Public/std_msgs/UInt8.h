#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class UInt8 : public FROSBaseMsg {
		public:
			UInt8() : UInt8(0) {}

			UInt8(uint8 data) {
				_MessageType = "std_msgs/UInt8";
				_Data = data;
			}

		//private:
			uint8 _Data;
		};
	}
}
