#pragma once

#include "ROSBaseMsg.h"
#include "ROSTime.h"


namespace ROSMessages{
	namespace std_msgs {
		class Header: public FROSBaseMsg {
		public:
			Header() {
				_MessageType = "std_msgs/Header";
			}

			Header(uint32 seq, FROSTime time, FString frame_id) : seq(seq), time(time), frame_id(frame_id) {
				_MessageType = "std_msgs/Header";
			}

			uint32 seq;
			FROSTime time;
			FString frame_id;
		};
	}
}
