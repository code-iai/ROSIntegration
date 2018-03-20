#pragma once 

#include "ROSBaseMsg.h"

namespace ROSMessages{
	namespace std_msgs {
		class Float32 : public FROSBaseMsg {
		public:
            Float32() {
				_MessageType = "std_msgs/Float32";
                _Data = 0.0f;
			}

            Float32(float data) {
				_MessageType = "std_msgs/Float32";
				_Data = data;
			}

		//private:
			float _Data;
		};


	}
}
