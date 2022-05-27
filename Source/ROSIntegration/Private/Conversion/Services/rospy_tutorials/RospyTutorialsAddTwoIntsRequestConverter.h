#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseRequestConverter.h"

#include "RospyTutorialsAddTwoIntsRequestConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API URospyTutorialsAddTwoIntsRequestConverter: public UBaseRequestConverter
{
	GENERATED_BODY()

public:
	URospyTutorialsAddTwoIntsRequestConverter();
	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) override;
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) override;

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest() override;
};