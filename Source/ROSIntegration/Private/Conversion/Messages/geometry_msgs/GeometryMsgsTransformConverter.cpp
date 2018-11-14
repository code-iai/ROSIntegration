#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformConverter.h"


UGeometryMsgsTransformConverter::UGeometryMsgsTransformConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Transform";
}

bool UGeometryMsgsTransformConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	UE_LOG(LogROS, Warning, TEXT("ROSIntegration: Transform receiving not implemented yet"));
	return false;
}

bool UGeometryMsgsTransformConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Transform = StaticCastSharedPtr<ROSMessages::geometry_msgs::Transform>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_transform(*message, Transform.Get());

	return true;
}
