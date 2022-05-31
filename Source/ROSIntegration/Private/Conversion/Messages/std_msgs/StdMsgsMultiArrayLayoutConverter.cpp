#include "StdMsgsMultiArrayLayoutConverter.h"


UStdMsgsMultiArrayLayoutConverter::UStdMsgsMultiArrayLayoutConverter()
{
	_MessageType = "std_msgs/MultiArrayLayout";
}

bool UStdMsgsMultiArrayLayoutConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::std_msgs::MultiArrayLayout;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_multi_array_layout(message->full_msg_bson_, "msg", msg);
}

bool UStdMsgsMultiArrayLayoutConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::std_msgs::MultiArrayLayout>(BaseMsg);
	*message = bson_new();
	_bson_append_multi_array_layout(*message, CastMsg.Get());
	return true;
}