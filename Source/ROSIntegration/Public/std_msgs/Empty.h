#pragma once

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class String : public FROSBaseMsg {
		public:
			String() {
				_MessageType = "std_msgs/Empty";
			}

			String(FString data) {
				_MessageType = "std_msgs/Empty";
			}

		//private:
		};
	}
}
