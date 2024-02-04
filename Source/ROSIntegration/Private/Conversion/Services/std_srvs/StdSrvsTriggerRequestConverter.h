#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseRequestConverter.h"

#include "StdSrvsTriggerRequestConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdSrvsTriggerRequestConverter: public UBaseRequestConverter
{
	GENERATED_BODY()

public:
	UStdSrvsTriggerRequestConverter();
	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) override;
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) override;

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest() override;
};