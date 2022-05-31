#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformConverter.h"


UGeometryMsgsTransformConverter::UGeometryMsgsTransformConverter()
{
	_MessageType = "geometry_msgs/Transform";
}

bool UGeometryMsgsTransformConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
    auto msg = new ROSMessages::geometry_msgs::Transform;
    BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
    return _bson_extract_child_transform(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsTransformConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::Transform>(BaseMsg);
	*message = bson_new();
	_bson_append_transform(*message, CastMsg.Get());
	return true;
}