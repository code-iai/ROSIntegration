#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformStampedConverter.h"

#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "geometry_msgs/TransformStamped.h"


UGeometryMsgsTransformStampedConverter::UGeometryMsgsTransformStampedConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/TransformStamped";
}

bool UGeometryMsgsTransformStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	UE_LOG(LogROS, Warning, TEXT("ROSIntegration: TransformStamped receiving not implemented yet"));
	return false;
}

bool UGeometryMsgsTransformStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto TransformStamped = StaticCastSharedPtr<ROSMessages::geometry_msgs::TransformStamped>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	UStdMsgsHeaderConverter::_bson_append_header(*message, &(TransformStamped->header));
	BSON_APPEND_UTF8(*message, "child_frame_id", TCHAR_TO_UTF8(*TransformStamped->child_frame_id));
	UGeometryMsgsTransformConverter::_bson_append_child_transform(*message, "transform", &(TransformStamped->transform));

	return true;
}
