#pragma once

#include <CoreMinimal.h>
#include "ROSBaseServiceRequest.h"

namespace std_srvs {
	class ROSINTEGRATION_API FTriggerRequest : public FROSBaseServiceRequest {

	public:
		FTriggerRequest() = default;
		~FTriggerRequest() = default;
		
	};
}
