#include "Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"

#include "geometry_msgs/Point.h"

UGeometryMsgsPointConverter::UGeometryMsgsPointConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Point";
}

bool UGeometryMsgsPointConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::Point();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_point(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsPointConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Point = StaticCastSharedPtr<ROSMessages::geometry_msgs::Point>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_point(*message, Point.Get());

	return true;
}
