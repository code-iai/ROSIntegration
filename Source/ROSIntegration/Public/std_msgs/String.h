#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class String : public FROSBaseMsg {
		public:
			String() {
				_MessageType = "std_msgs/String";
			}

			String(FString data) {
				_MessageType = "std_msgs/String";
				_Data = data;
			}

		//private:
			FString _Data;
		};
	}
}
