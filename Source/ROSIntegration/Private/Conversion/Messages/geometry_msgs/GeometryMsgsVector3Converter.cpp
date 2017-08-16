#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"


UGeometryMsgsVector3Converter::UGeometryMsgsVector3Converter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Vector3";
}

bool UGeometryMsgsVector3Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	return true;
}

bool UGeometryMsgsVector3Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	return true;
}