#pragma once
#include <chrono>

class ROSINTEGRATION_API FROSTime {
public:
	FROSTime() : _Sec(0), _NSec(0) {}

	FROSTime(unsigned long Sec, unsigned long Nsec) : _Sec(Sec), _NSec(Nsec) {}

	static FROSTime Now();

	static void SetUseSimTime(bool bUseSimTime);
	static void SetSimTime(const FROSTime& time);

	unsigned long _Sec;
	unsigned long _NSec;
};
