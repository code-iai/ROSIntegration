#pragma once

#include "CoreMinimal.h"
//#include "ROSBaseMsg.generated.h"

class ROSINTEGRATION_API FROSBaseServiceResponse {

public:
	FROSBaseServiceResponse() = default;
	~FROSBaseServiceResponse() = default;

	bool _Result;
};