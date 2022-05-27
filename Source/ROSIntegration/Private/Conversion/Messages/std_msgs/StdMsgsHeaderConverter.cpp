#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"


UStdMsgsHeaderConverter::UStdMsgsHeaderConverter()
{
	_MessageType = "std_msgs/Header";
}

bool UStdMsgsHeaderConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::std_msgs::Header();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_header(message->full_msg_bson_, "msg", msg);
}

bool UStdMsgsHeaderConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::std_msgs::Header>(BaseMsg);
	*message = bson_new();
	_bson_append_header(*message, CastMsg.Get());
	return true;
}