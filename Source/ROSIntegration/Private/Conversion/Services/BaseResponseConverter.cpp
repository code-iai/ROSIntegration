#include "Conversion/Services/BaseResponseConverter.h"


UBaseResponseConverter::UBaseResponseConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
}

bool UBaseResponseConverter::ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response)
{
	return false;
}

bool UBaseResponseConverter::ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res)
{
	return false;
}

TSharedPtr<FROSBaseServiceResponse> UBaseResponseConverter::AllocateConcreteResponse()
{
	return TSharedPtr<FROSBaseServiceResponse>();
}
