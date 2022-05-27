#include "Conversion/Services/BaseRequestConverter.h"


UBaseRequestConverter::UBaseRequestConverter()
{
}


bool UBaseRequestConverter::ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest)
{
	return false;
}

bool UBaseRequestConverter::ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request)
{
	return false;
}

TSharedPtr<FROSBaseServiceRequest> UBaseRequestConverter::AllocateConcreteRequest()
{
	return TSharedPtr<FROSBaseServiceRequest>();
}
