#include "Conversion/Messages/std_msgs/StdMsgsUInt8MultiArrayConverter.h"

UStdMsgsUInt8MultiArrayConverter::UStdMsgsUInt8MultiArrayConverter()
{
	_MessageType = "std_msgs/UInt8MultiArray";
}

bool UStdMsgsUInt8MultiArrayConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::std_msgs::UInt8MultiArray;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_uint8_multi_array(message->full_msg_bson_, "msg", msg);
}

bool UStdMsgsUInt8MultiArrayConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::std_msgs::UInt8MultiArray>(BaseMsg);
	*message = bson_new();
	_bson_append_uint8_multi_array(*message, CastMsg.Get());
	return true;
}