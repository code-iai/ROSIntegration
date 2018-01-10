#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformStampedConverter.h"

#include "geometry_msgs/TransformStamped.h"


UGeometryMsgsTransformStampedConverter::UGeometryMsgsTransformStampedConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/TransformStamped";
}

bool UGeometryMsgsTransformStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	return true;
}

bool UGeometryMsgsTransformStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	
	auto TransformStamped = StaticCastSharedPtr<ROSMessages::geometry_msgs::TransformStamped>(BaseMsg);

	bson_init(*message);
	_bson_append_header(*message, TransformStamped->header);
	BSON_APPEND_UTF8(*message, "child_frame_id", TCHAR_TO_UTF8(*TransformStamped->child_frame_id));
	UGeometryMsgsTransformConverter::_bson_append_child_transform(*message, "transform", &(TransformStamped->transform));

	return true;
}