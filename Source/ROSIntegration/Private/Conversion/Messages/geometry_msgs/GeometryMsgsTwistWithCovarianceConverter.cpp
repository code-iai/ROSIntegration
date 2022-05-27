#include "GeometryMsgsTwistWithCovarianceConverter.h"


UGeometryMsgsTwistWithCovarianceConverter::UGeometryMsgsTwistWithCovarianceConverter()
{
	_MessageType = "geometry_msgs/TwistWithCovariance";
}

bool UGeometryMsgsTwistWithCovarianceConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geometry_msgs::TwistWithCovariance();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_twist_with_covariance(message->full_msg_bson_, "msg", msg);
}

bool UGeometryMsgsTwistWithCovarianceConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::TwistWithCovariance>(BaseMsg);
	*message = bson_new();
	_bson_append_twist_with_covariance(*message, CastMsg.Get());
	return true;
}