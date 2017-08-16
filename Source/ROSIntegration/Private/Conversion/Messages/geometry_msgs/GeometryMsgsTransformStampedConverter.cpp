#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformStampedConverter.h"


UGeometryMsgsTransformStampedConverter::UGeometryMsgsTransformStampedConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/TransformStamped";
}

bool UGeometryMsgsTransformStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	return true;
}

bool UGeometryMsgsTransformStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	return true;
}