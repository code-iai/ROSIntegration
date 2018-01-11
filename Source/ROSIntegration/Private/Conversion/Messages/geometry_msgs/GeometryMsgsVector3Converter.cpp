#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"

#include "geometry_msgs/Vector3.h"

UGeometryMsgsVector3Converter::UGeometryMsgsVector3Converter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Vector3";
}

bool UGeometryMsgsVector3Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	UE_LOG(LogTemp, Warning, TEXT("ROSIntegration: Vector3 receiving not implemented yet"));
	return false;
}

bool UGeometryMsgsVector3Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	
	auto Vector3 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(BaseMsg);

	bson_init(*message);
	_bson_append_vector3(*message, Vector3.Get());
	
	return true;
}