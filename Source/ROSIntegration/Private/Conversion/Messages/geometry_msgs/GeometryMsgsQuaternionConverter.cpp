#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"


UGeometryMsgsQuaternionConverter::UGeometryMsgsQuaternionConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Quaternion";
}

bool UGeometryMsgsQuaternionConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	return true;
}

bool UGeometryMsgsQuaternionConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	return true;
}