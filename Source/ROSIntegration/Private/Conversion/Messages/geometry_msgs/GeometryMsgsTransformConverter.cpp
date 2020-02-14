#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformConverter.h"


UGeometryMsgsTransformConverter::UGeometryMsgsTransformConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Transform";
}

bool UGeometryMsgsTransformConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
//    UE_LOG(LogTemp, Warning, TEXT("ROSIntegration: Transform receiving not implemented yet")); // TODO: use ROS log
    UE_LOG(LogTemp, Warning, TEXT("ROSIntegration: Transform received"));
    auto p = new ROSMessages::geometry_msgs::Transform;
    BaseMsg = TSharedPtr<FROSBaseMsg>(p);
    return (_bson_extract_child_transform(message->full_msg_bson_, "msg", p));
}

bool UGeometryMsgsTransformConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Transform = StaticCastSharedPtr<ROSMessages::geometry_msgs::Transform>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_transform(*message, Transform.Get());

	return true;
}
