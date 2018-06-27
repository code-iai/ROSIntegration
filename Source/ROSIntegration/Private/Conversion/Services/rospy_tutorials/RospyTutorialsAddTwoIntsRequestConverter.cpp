#include "Conversion/Services/rospy_tutorials/RospyTutorialsAddTwoIntsRequestConverter.h"
#include "rospy_tutorials/AddTwoIntsRequest.h"


URospyTutorialsAddTwoIntsRequestConverter::URospyTutorialsAddTwoIntsRequestConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_ServiceType = "rospy_tutorials/AddTwoInts";
}

bool URospyTutorialsAddTwoIntsRequestConverter::ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest)
{
	auto AddTwoIntsRequest = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsRequest>(Request);
	*BSONRequest = BCON_NEW(
		"a", BCON_INT32(AddTwoIntsRequest->_a),
		"b", BCON_INT32(AddTwoIntsRequest->_b)
	);

	UE_LOG(LogTemp, Verbose, TEXT("This is rospy tut req converter"));
	return true;
}

bool URospyTutorialsAddTwoIntsRequestConverter::ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request)
{
	//*Request = MakeShareable(new rospy_tutorials::FAddTwoIntsRequest);
	auto ServiceRequest = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsRequest>(Request);
	bool key_found = false;

	// Retrieve attributes from wire-representation of class
	ServiceRequest->_a = rosbridge2cpp::Helper::get_int32_by_key("args.a", *(req.full_msg_bson_), key_found);
	if (!key_found) {
		UE_LOG(LogTemp, Error, TEXT("Key args.a not present in data"));
		return false;
	}
	UE_LOG(LogTemp, Verbose, TEXT("Request.a is %d"), ServiceRequest->_a);

	ServiceRequest->_b = rosbridge2cpp::Helper::get_int32_by_key("args.b", *(req.full_msg_bson_), key_found);
	if (!key_found) {
		UE_LOG(LogTemp, Error, TEXT("Key args.b not present in data"));
		return false;
	}
	UE_LOG(LogTemp, Verbose, TEXT("Request.b is %d"), ServiceRequest->_b);

	return true;
}

TSharedPtr<FROSBaseServiceRequest> URospyTutorialsAddTwoIntsRequestConverter::AllocateConcreteRequest()
{
	return MakeShareable(new rospy_tutorials::FAddTwoIntsRequest);
}
