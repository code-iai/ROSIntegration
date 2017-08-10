#pragma once

#include "CoreMinimal.h"
//#include "ROSBaseMsg.generated.h"
namespace rospy_tutorials {
	class ROSINTEGRATION_API FAddTwoIntsRequest : public FROSBaseServiceRequest{

	public:
		FAddTwoIntsRequest() = default;
		~FAddTwoIntsRequest() = default;

		int64 _a, _b;
	};
}
