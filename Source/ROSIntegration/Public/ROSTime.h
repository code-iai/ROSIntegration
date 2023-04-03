#pragma once
#include <chrono>

class ROSINTEGRATION_API FROSTime {
public:
	FROSTime() : _Sec(0), _NSec(0) {}

	FROSTime(unsigned long Sec, unsigned long Nsec) : _Sec(Sec), _NSec(Nsec) {}

	static FROSTime Now();

	static void SetUseSimTime(bool bUseSimTime);
	static void SetSimTime(const FROSTime& time);

	// Get the time difference from Time1 to Time2
	static double GetTimeDelta(const FROSTime& Time1, const FROSTime& Time2);

	unsigned long _Sec;
	unsigned long _NSec;
};
