#include "Conversion/Messages/nav_msgs/NavMsgsPathConverter.h"


UNavMsgsPathConverter::UNavMsgsPathConverter()
{
	_MessageType = "nav_msgs/Path";
}

bool UNavMsgsPathConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::nav_msgs::Path();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_path(message->full_msg_bson_, "msg", msg);
}

bool UNavMsgsPathConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::nav_msgs::Path>(BaseMsg);
	*message = bson_new();
	_bson_append_path(*message, CastMsg.Get());
	return true;
}