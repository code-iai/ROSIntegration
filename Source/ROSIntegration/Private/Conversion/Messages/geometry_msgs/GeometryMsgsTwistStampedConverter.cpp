#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistStampedConverter.h"


UGeometryMsgsTwistStampedConverter::UGeometryMsgsTwistStampedConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/TwistStamped";
}

bool UGeometryMsgsTwistStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::TwistStamped();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_twist_stamped(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsTwistStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto TwistStamped = StaticCastSharedPtr<ROSMessages::geometry_msgs::TwistStamped>(BaseMsg);

	*message = bson_new();
	_bson_append_twist_stamped(*message, TwistStamped.Get());

	return true;
}
