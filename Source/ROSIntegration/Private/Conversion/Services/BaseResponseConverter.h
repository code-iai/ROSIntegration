#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "rosbridge2cpp/messages/rosbridge_service_response_msg.h"
#include "ROSBaseServiceResponse.h"

#include "BaseResponseConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UBaseResponseConverter: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY()
	FString _ServiceType;

	/// This method will be used to convert the Response an external service to the UnrealRI format
	virtual bool ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response);

	/// This method will be used to convert the Response from an self-advertised service to the rosbrige2cpp format
	virtual bool ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res);

	virtual TSharedPtr<FROSBaseServiceResponse> AllocateConcreteResponse();
};
