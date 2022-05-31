#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"


UGeometryMsgsQuaternionConverter::UGeometryMsgsQuaternionConverter()
{
	_MessageType = "geometry_msgs/Quaternion";
}

bool UGeometryMsgsQuaternionConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::Quaternion();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_quaternion(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsQuaternionConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::Quaternion>(BaseMsg);
	*message = bson_new();
	_bson_append_quaternion(*message, CastMsg.Get());
	return true;
}