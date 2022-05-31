#pragma once

#include "CoreMinimal.h"
#include "rosbridge2cpp/messages/rosbridge_service_response_msg.h"
#include "ROSBaseServiceResponse.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "BaseResponseConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UBaseResponseConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UBaseResponseConverter();

	UPROPERTY()
	FString _ServiceType;

	/// This method will be used to convert the Response an external service to the UnrealRI format
	virtual bool ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response);

	/// This method will be used to convert the Response from an self-advertised service to the rosbrige2cpp format
	virtual bool ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res);

	virtual TSharedPtr<FROSBaseServiceResponse> AllocateConcreteResponse();
};