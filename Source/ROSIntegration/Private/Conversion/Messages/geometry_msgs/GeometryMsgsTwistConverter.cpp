#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistConverter.h"


UGeometryMsgsTwistConverter::UGeometryMsgsTwistConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Twist";
}

bool UGeometryMsgsTwistConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::Twist();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_twist(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsTwistConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Twist = StaticCastSharedPtr<ROSMessages::geometry_msgs::Twist>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_twist(*message, Twist.Get());

	return true;
}
