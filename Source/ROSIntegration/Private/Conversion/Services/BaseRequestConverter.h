#pragma once

#include "CoreMinimal.h"
#include "rosbridge2cpp/messages/rosbridge_call_service_msg.h"
#include "ROSBaseServiceRequest.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "BaseRequestConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UBaseRequestConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UBaseRequestConverter();

	UPROPERTY()
	FString _ServiceType;

	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request);
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest);

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest();
};