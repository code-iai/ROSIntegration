#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformConverter.h"


UGeometryMsgsTransformConverter::UGeometryMsgsTransformConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Transform";
}

bool UGeometryMsgsTransformConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	return true;
}

bool UGeometryMsgsTransformConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {

	auto Transform = StaticCastSharedPtr<ROSMessages::geometry_msgs::Transform>(BaseMsg);

	bson_init(*message);
	_bson_append_transform(*message, Transform.Get());

	return true;
}