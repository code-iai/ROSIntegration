#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"

UGeometryMsgsVector3Converter::UGeometryMsgsVector3Converter()
{
	_MessageType = "geometry_msgs/Vector3";
}

bool UGeometryMsgsVector3Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::Vector3();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_vector3(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsVector3Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(BaseMsg);
	*message = bson_new();
	_bson_append_vector3(*message, CastMsg.Get());
	return true;
}