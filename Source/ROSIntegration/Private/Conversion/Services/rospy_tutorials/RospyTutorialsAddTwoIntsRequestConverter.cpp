#include "Conversion/Services/rospy_tutorials/RospyTutorialsAddTwoIntsRequestConverter.h"
#include "rospy_tutorials/AddTwoIntsRequest.h"


URospyTutorialsAddTwoIntsRequestConverter::URospyTutorialsAddTwoIntsRequestConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_ServiceType = "rospy_tutorials/AddTwoInts";
}

bool URospyTutorialsAddTwoIntsRequestConverter::ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) {

	auto AddTwoIntsRequest = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsRequest>(Request);
	*BSONRequest = BCON_NEW(
		"a", BCON_INT32(AddTwoIntsRequest->_a),
		"b", BCON_INT32(AddTwoIntsRequest->_b)
	);

	UE_LOG(LogTemp, Warning, TEXT("This is rospy tut req converter"));
	return true;
}