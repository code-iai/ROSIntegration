#include "ROSGraphMsgsClockConverter.h"


UROSGraphMsgsClockConverter::UROSGraphMsgsClockConverter()
{
	_MessageType = "rosgraph_msgs/Clock";
}

bool UROSGraphMsgsClockConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::rosgraph_msgs::Clock();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_clock(message->full_msg_bson_, "msg", msg);
}

bool UROSGraphMsgsClockConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::rosgraph_msgs::Clock>(BaseMsg);
	*message = bson_new();
	_bson_append_clock(*message, CastMsg.Get());
	return true;
}