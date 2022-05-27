#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"


UGeometryMsgsPoseConverter::UGeometryMsgsPoseConverter()
{
	_MessageType = "geometry_msgs/Pose";
}

bool UGeometryMsgsPoseConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::Pose();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_pose(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsPoseConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMSG = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(BaseMsg);
	*message = bson_new();
	_bson_append_pose(*message, CastMSG.Get());
	return true;
}
