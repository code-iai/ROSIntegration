#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseResponseConverter.h"
#include "StdSrvsTriggerResponseConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdSrvsTriggerResponseConverter: public UBaseResponseConverter
{
	GENERATED_BODY()

public:
	UStdSrvsTriggerResponseConverter();
	virtual bool ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response) override;
	virtual bool ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res) override;

	virtual TSharedPtr<FROSBaseServiceResponse> AllocateConcreteResponse() override;
};