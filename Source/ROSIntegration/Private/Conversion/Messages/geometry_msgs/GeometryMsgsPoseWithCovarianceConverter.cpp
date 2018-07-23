#include "GeometryMsgsPoseWithCovarianceConverter.h"


UGeometryMsgsPoseWithCovarianceConverter::UGeometryMsgsPoseWithCovarianceConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/PoseWithCovariance";
}

bool UGeometryMsgsPoseWithCovarianceConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::PoseWithCovariance();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_pose_with_covariance(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsPoseWithCovarianceConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Pose = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseWithCovariance>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_pose_with_covariance(*message, Pose.Get());

	return true;
}
