#pragma once

#include <CoreMinimal.h>
#include "ROSBaseServiceResponse.h"

namespace std_srvs {
	class ROSINTEGRATION_API FTriggerResponse : public FROSBaseServiceResponse {

	public:
		FTriggerResponse() = default;
		~FTriggerResponse() = default;

		bool _success;
		FString _message;
	};
}
