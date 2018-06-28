#include "Conversion/Services/rospy_tutorials/RospyTutorialsAddTwoIntsResponseConverter.h"

#include "rospy_tutorials/AddTwoIntsResponse.h"
#include <bson.h>


URospyTutorialsAddTwoIntsResponseConverter::URospyTutorialsAddTwoIntsResponseConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_ServiceType = "rospy_tutorials/AddTwoInts";
}

bool URospyTutorialsAddTwoIntsResponseConverter::ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response)
{
	bool key_found = false;
	*Response = MakeShareable(new rospy_tutorials::FAddTwoIntsResponse);
	auto ServiceResponse = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsResponse>(*Response);

	ServiceResponse->_Result = res.result_;

	ServiceResponse->_sum = rosbridge2cpp::Helper::get_int32_by_key("values.sum", *res.full_msg_bson_, key_found);
	if (!key_found) {
		UE_LOG(LogTemp, Error, TEXT("Key values.sum not present in data"));
		return false;
	}
	return true;
}

bool URospyTutorialsAddTwoIntsResponseConverter::ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res)
{
	auto ServiceResponse = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsResponse>(Response);

	res.result_ = ServiceResponse->_Result;
	BSON_APPEND_INT32(res.values_bson_, "sum", ServiceResponse->_sum);
	return true;
}

TSharedPtr<FROSBaseServiceResponse> URospyTutorialsAddTwoIntsResponseConverter::AllocateConcreteResponse()
{
	return MakeShareable(new rospy_tutorials::FAddTwoIntsResponse);
}
