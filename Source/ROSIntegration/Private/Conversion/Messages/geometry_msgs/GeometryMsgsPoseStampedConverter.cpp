#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseStampedConverter.h"


UGeometryMsgsPoseStampedConverter::UGeometryMsgsPoseStampedConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/PoseStamped";
}

bool UGeometryMsgsPoseStampedConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::PoseStamped();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);

	if (!UStdMsgsHeaderConverter::_bson_extract_child_header(message->full_msg_bson_, TEXT("msg.header"), &p->header)) return false;
	if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(message->full_msg_bson_, TEXT("msg.pose"), &p->pose)) return false;

	return true;
}

bool UGeometryMsgsPoseStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto PoseStamped = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);

	_bson_append_pose_stamped(*message, PoseStamped.Get());

	return true;
}
