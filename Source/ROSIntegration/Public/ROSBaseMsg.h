#pragma once

#include "CoreMinimal.h"
//#include "ROSBaseMsg.generated.h"

class ROSINTEGRATION_API FROSBaseMsg {

public:
	FROSBaseMsg() = default;
	~FROSBaseMsg() = default;

	//protected: 
		FString _MessageType;
};