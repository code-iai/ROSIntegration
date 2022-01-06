#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class Bool : public FROSBaseMsg {
		public:
			Bool() : Bool(false) {}

			Bool(bool data) {
				_MessageType = "std_msgs/Bool";
				_Data = data;
			}

		//private:
			bool _Data;
		};
	}
}
