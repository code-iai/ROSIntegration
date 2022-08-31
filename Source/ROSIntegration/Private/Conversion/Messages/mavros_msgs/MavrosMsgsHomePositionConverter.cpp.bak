#include "Conversion/Messages/mavros_msgs/MavrosMsgsHomePositionConverter.h"


UMavrosMsgsHomePositionConverter::UMavrosMsgsHomePositionConverter()
{
	_MessageType = "mavros_msgs/HomePosition";
}

bool UMavrosMsgsHomePositionConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::mavros_msgs::HomePosition();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);

	if (!UStdMsgsHeaderConverter::_bson_extract_child_header(message->full_msg_bson_, TEXT("msg.header"), &msg->header)) return false;
	if (!UGeometryMsgsPointConverter::_bson_extract_child_point(message->full_msg_bson_, TEXT("msg.position"), &msg->position)) return false;
	if (!UGeometryMsgsQuaternionConverter::_bson_extract_child_quaternion(message->full_msg_bson_, TEXT("msg.orientation"), &msg->orientation)) return false;
	if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(message->full_msg_bson_, TEXT("msg.approach"), &msg->approach)) return false;
	if (!UGeographicMsgsGeoPointConverter::_bson_extract_child_geo_point(message->full_msg_bson_, TEXT("msg.geo"), &msg->geo)) return false;

	return true;
}

bool UMavrosMsgsHomePositionConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMSG = StaticCastSharedPtr<ROSMessages::mavros_msgs::HomePosition>(BaseMsg);
	*message = bson_new();
	_bson_append_home_position(*message, CastMSG.Get());

	return true;
}
