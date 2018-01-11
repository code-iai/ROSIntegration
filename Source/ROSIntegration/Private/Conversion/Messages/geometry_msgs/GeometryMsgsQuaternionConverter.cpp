#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"


UGeometryMsgsQuaternionConverter::UGeometryMsgsQuaternionConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Quaternion";
}

bool UGeometryMsgsQuaternionConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	UE_LOG(LogTemp, Warning, TEXT("ROSIntegration: Quaternion receiving not implemented yet"));
	return false;
}

bool UGeometryMsgsQuaternionConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {

	auto Quaternion = StaticCastSharedPtr<ROSMessages::geometry_msgs::Quaternion>(BaseMsg);

	bson_init(*message);
	_bson_append_quaternion(*message, Quaternion.Get());

	return true;
}