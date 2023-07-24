#include "Conversion/Messages/std_msgs/StdMsgsColorRGBAConverter.h"


UStdMsgsColorRGBAConverter::UStdMsgsColorRGBAConverter()
{
	_MessageType = "std_msgs/ColorRGBA";
}

bool UStdMsgsColorRGBAConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::std_msgs::ColorRGBA();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_color_rgba(message->full_msg_bson_, "msg", msg);
}

bool UStdMsgsColorRGBAConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::std_msgs::ColorRGBA>(BaseMsg);
	*message = bson_new();
	_bson_append_color_rgba(*message, CastMsg.Get());
	return true;
}