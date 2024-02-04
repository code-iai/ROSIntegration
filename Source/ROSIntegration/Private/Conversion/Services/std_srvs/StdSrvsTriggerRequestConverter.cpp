#include "Conversion/Services/std_srvs/StdSrvsTriggerRequestConverter.h"
#include "std_srvs/TriggerRequest.h"


UStdSrvsTriggerRequestConverter::UStdSrvsTriggerRequestConverter()
{
	_ServiceType = "std_srvs/Trigger";
}

bool UStdSrvsTriggerRequestConverter::ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest)
{
	auto TriggerRequest = StaticCastSharedPtr<std_srvs::FTriggerRequest>(Request);
	*BSONRequest = BCON_NEW();

	return true;
}

bool UStdSrvsTriggerRequestConverter::ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request)
{
	//*Request = MakeShareable(new std_srvs::FTriggerRequest);
	auto ServiceRequest = StaticCastSharedPtr<std_srvs::FTriggerRequest>(Request);

	return true;
}

TSharedPtr<FROSBaseServiceRequest> UStdSrvsTriggerRequestConverter::AllocateConcreteRequest()
{
	return MakeShareable(new std_srvs::FTriggerRequest);
}