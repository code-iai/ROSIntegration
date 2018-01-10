#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"

#include "geometry_msgs/Vector3.h"

UGeometryMsgsVector3Converter::UGeometryMsgsVector3Converter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Vector3";
}

bool UGeometryMsgsVector3Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	return true;
}

bool UGeometryMsgsVector3Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	
	auto Vector3 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(BaseMsg);

	bson_init(*message);
	BSON_APPEND_DOUBLE(*message, "x", Vector3->x);
	BSON_APPEND_DOUBLE(*message, "y", Vector3->x);
	BSON_APPEND_DOUBLE(*message, "z", Vector3->x);
	
	return true;
}