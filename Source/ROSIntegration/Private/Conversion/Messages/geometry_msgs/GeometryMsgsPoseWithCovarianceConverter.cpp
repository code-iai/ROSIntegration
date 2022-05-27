#include "GeometryMsgsPoseWithCovarianceConverter.h"


UGeometryMsgsPoseWithCovarianceConverter::UGeometryMsgsPoseWithCovarianceConverter()
{
	_MessageType = "geometry_msgs/PoseWithCovariance";
}

bool UGeometryMsgsPoseWithCovarianceConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::PoseWithCovariance();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_pose_with_covariance(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsPoseWithCovarianceConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseWithCovariance>(BaseMsg);
	*message = bson_new();
	_bson_append_pose_with_covariance(*message, CastMsg.Get());
	return true;
}