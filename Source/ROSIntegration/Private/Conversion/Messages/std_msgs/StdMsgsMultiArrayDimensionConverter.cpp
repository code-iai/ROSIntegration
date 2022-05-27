#include "Conversion/Messages/std_msgs/StdMsgsMultiArrayDimensionConverter.h"

UStdMsgsMultiArrayDimensionConverter::UStdMsgsMultiArrayDimensionConverter()
{
	_MessageType = "std_msgs/MultiArrayDimension";
}

bool UStdMsgsMultiArrayDimensionConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::std_msgs::MultiArrayDimension;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_multi_array_dimension(message->full_msg_bson_, "msg", msg);
}

bool UStdMsgsMultiArrayDimensionConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::std_msgs::MultiArrayDimension>(BaseMsg);
	*message = bson_new();
	_bson_append_multi_array_dimension(*message, CastMsg.Get());
	return true;
}