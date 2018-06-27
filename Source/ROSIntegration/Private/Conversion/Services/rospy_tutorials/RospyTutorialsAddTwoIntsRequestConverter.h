#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Services/BaseRequestConverter.h"

#include "RospyTutorialsAddTwoIntsRequestConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API URospyTutorialsAddTwoIntsRequestConverter: public UBaseRequestConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) override;
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) override;

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest() override;
};
