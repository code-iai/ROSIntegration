#include "Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"

#include "geometry_msgs/Point.h"

UGeometryMsgsPointConverter::UGeometryMsgsPointConverter()
{
	_MessageType = "geometry_msgs/Point";
}

bool UGeometryMsgsPointConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::Point();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_point(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsPointConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMSG = StaticCastSharedPtr<ROSMessages::geometry_msgs::Point>(BaseMsg);
	*message = bson_new();
	_bson_append_point(*message, CastMSG.Get());
	return true;
}
