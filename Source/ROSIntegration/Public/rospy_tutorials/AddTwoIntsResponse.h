#pragma once

#include "CoreMinimal.h"
//#include "ROSBaseMsg.generated.h"

namespace rospy_tutorials {
	class ROSINTEGRATION_API FAddTwoIntsResponse : public FROSBaseServiceResponse {

	public:
		FAddTwoIntsResponse() = default;
		~FAddTwoIntsResponse() = default;

		int64 _sum;
	};
}
