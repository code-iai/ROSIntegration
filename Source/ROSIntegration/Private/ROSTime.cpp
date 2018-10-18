#include "ROSTime.h"
#include "ros_time.h"

using rosbridge2cpp::ROSTime;

FROSTime FROSTime::Now() {
	if (ROSTime::use_sim_time) {
		return FROSTime(ROSTime::sim_time.sec_, ROSTime::sim_time.nsec_);
	}

	const std::chrono::high_resolution_clock::duration time_since_epoch = ROSTime::HRCEpocOffset + std::chrono::high_resolution_clock::now().time_since_epoch();
	unsigned long seconds_since_epoch =	std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch).count();
	unsigned long long nanoseconds_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch).count();
	unsigned long nanosecond_difference = nanoseconds_since_epoch - (seconds_since_epoch * 1000000000ul);
	return FROSTime(seconds_since_epoch, nanosecond_difference);
}

void FROSTime::SetUseSimTime(bool bUseSimTime)
{
	ROSTime::use_sim_time = bUseSimTime;
}

void FROSTime::SetSimTime(const FROSTime& time)
{
	ROSTime::sim_time.sec_ = time._Sec;
	ROSTime::sim_time.nsec_ = time._NSec;
}
