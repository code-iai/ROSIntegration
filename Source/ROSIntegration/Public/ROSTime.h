#pragma once 
#include <chrono>

class FROSTime {
public:
	FROSTime() : _Sec(0), _NSec(0) {}

	FROSTime(unsigned long Sec, unsigned long Nsec) : _Sec(Sec), _NSec(Nsec) {}

	static FROSTime Now() {
		unsigned long seconds_since_epoch =
			std::chrono::duration_cast<std::chrono::seconds>
			(std::chrono::system_clock::now().time_since_epoch()).count();
		unsigned long long nanoseconds_since_epoch =
			std::chrono::duration_cast<std::chrono::nanoseconds>
			(std::chrono::system_clock::now().time_since_epoch()).count();
		unsigned long nanosecond_difference = nanoseconds_since_epoch - (seconds_since_epoch * 1000000000ul);
		return FROSTime(seconds_since_epoch, nanosecond_difference);
	}

	unsigned long _Sec;
	unsigned long _NSec;
};
