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

	bool KeyFound = false;
	KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(message->full_msg_bson_, TEXT("msg.header"), &p->header); if (!KeyFound) return false;
	KeyFound = UGeometryMsgsPoseConverter::_bson_extract_child_pose(message->full_msg_bson_, TEXT("msg.pose"), &p->pose); if (!KeyFound) return false;

	return true;
}

bool UGeometryMsgsPoseStampedConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
	auto PoseStamped = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);

	UStdMsgsHeaderConverter::_bson_append_header(*message, &(PoseStamped->header));
	UGeometryMsgsPoseConverter::_bson_append_child_pose(*message, "pose", &(PoseStamped->pose));

	return true;
}