#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Services/BaseResponseConverter.h"

#include "RospyTutorialsAddTwoIntsResponseConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API URospyTutorialsAddTwoIntsResponseConverter: public UBaseResponseConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response) override;
	virtual bool ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res) override;

	virtual TSharedPtr<FROSBaseServiceResponse> AllocateConcreteResponse() override;
};
