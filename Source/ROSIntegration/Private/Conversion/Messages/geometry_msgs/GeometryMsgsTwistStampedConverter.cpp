#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistStampedConverter.h"


UGeometryMsgsTwistStampedConverter::UGeometryMsgsTwistStampedConverter()
{
	_MessageType = "geometry_msgs/TwistStamped";
}

bool UGeometryMsgsTwistStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::TwistStamped();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_twist_stamped(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsTwistStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::TwistStamped>(BaseMsg);
	*message = bson_new();
	_bson_append_twist_stamped(*message, CastMsg.Get());
	return true;
}