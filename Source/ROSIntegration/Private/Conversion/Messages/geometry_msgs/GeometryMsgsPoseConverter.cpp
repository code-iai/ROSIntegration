#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"


UGeometryMsgsPoseConverter::UGeometryMsgsPoseConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Pose";
}

bool UGeometryMsgsPoseConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::Pose();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_pose(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsPoseConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Pose = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_pose(*message, Pose.Get());

	return true;
}
