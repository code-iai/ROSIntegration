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

	UE_LOG(LogTemp, Verbose, TEXT("This is rospy tut req converter"));
	return true;
}

bool UStdSrvsTriggerRequestConverter::ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request)
{
	//*Request = MakeShareable(new std_srvs::FTriggerRequest);
	auto ServiceRequest = StaticCastSharedPtr<std_srvs::FTriggerRequest>(Request);
	
	UE_LOG(LogTemp, Verbose, TEXT("Received Trigger Request"));

	return true;
}

TSharedPtr<FROSBaseServiceRequest> UStdSrvsTriggerRequestConverter::AllocateConcreteRequest()
{
	return MakeShareable(new std_srvs::FTriggerRequest);
}