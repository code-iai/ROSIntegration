#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformStampedConverter.h"


UGeometryMsgsTransformStampedConverter::UGeometryMsgsTransformStampedConverter()
{
	_MessageType = "geometry_msgs/TransformStamped";
}

bool UGeometryMsgsTransformStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
    auto msg = new ROSMessages::geometry_msgs::TransformStamped;
    BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
    return _bson_extract_child_transform_stamped(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsTransformStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::TransformStamped>(BaseMsg);
	*message = bson_new();
	_bson_append_transform_stamped(*message, CastMsg.Get());
	return true;
}