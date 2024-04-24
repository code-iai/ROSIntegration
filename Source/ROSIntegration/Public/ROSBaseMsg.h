#pragma once

#include <CoreMinimal.h>
#include "ros_version.h"

class ROSINTEGRATION_API FROSBaseMsg {

public:
	FROSBaseMsg() = default;
	~FROSBaseMsg() = default;

	FString _MessageType;
};
