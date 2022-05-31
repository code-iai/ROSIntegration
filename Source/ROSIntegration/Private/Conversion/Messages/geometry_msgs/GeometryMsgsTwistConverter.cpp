#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistConverter.h"


UGeometryMsgsTwistConverter::UGeometryMsgsTwistConverter()
{
	_MessageType = "geometry_msgs/Twist";
}

bool UGeometryMsgsTwistConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::Twist();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_twist(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsTwistConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::Twist>(BaseMsg);
	*message = bson_new();
	_bson_append_twist(*message, CastMsg.Get());
	return true;
}