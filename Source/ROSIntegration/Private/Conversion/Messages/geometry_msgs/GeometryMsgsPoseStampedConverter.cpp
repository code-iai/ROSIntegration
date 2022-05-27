#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseStampedConverter.h"


UGeometryMsgsPoseStampedConverter::UGeometryMsgsPoseStampedConverter()
{
	_MessageType = "geometry_msgs/PoseStamped";
}

bool UGeometryMsgsPoseStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::PoseStamped();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_pose_stamped(message->full_msg_bson_, "msg", msg);;
}

bool UGeometryMsgsPoseStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(BaseMsg);
	*message = bson_new();
	_bson_append_pose_stamped(*message, CastMsg.Get());
	return true;
}