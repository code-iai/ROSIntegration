#include "Conversion/Messages/std_msgs/StdMsgsUInt8MultiArrayConverter.h"

UStdMsgsUInt8MultiArrayConverter::UStdMsgsUInt8MultiArrayConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/UInt8MultiArray";
}

bool UStdMsgsUInt8MultiArrayConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::std_msgs::UInt8MultiArray;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_uint8_multi_array(message->full_msg_bson_, "msg", p);
}

bool UStdMsgsUInt8MultiArrayConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto bma = StaticCastSharedPtr<ROSMessages::std_msgs::UInt8MultiArray>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_uint8_multi_array(*message, bma.Get());

	return true;
}
