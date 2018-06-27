#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "rosbridge2cpp/messages/rosbridge_call_service_msg.h"
#include "ROSBaseServiceRequest.h"
#include <bson.h>

#include "BaseRequestConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UBaseRequestConverter: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY()
	FString _ServiceType;

	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request);
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest);

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest();
};
