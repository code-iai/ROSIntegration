#pragma once

#include <CoreMinimal.h>

class ROSINTEGRATION_API FROSBaseMsg {

public:
	FROSBaseMsg() = default;
	~FROSBaseMsg() = default;

	FString _MessageType;
};
