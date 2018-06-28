#include "StdMsgsMultiArrayLayoutConverter.h"


UStdMsgsMultiArrayLayoutConverter::UStdMsgsMultiArrayLayoutConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/MultiArrayLayout";
}

bool UStdMsgsMultiArrayLayoutConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::std_msgs::MultiArrayLayout;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_multi_array_layout(message->full_msg_bson_, "msg", p);
}

bool UStdMsgsMultiArrayLayoutConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Pose = StaticCastSharedPtr<ROSMessages::std_msgs::MultiArrayLayout>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_multi_array_layout(*message, Pose.Get());

	return true;
}
