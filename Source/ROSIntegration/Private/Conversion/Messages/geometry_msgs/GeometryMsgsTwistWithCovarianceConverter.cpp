#include "GeometryMsgsTwistWithCovarianceConverter.h"


UGeometryMsgsTwistWithCovarianceConverter::UGeometryMsgsTwistWithCovarianceConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/TwistWithCovariance";
}

bool UGeometryMsgsTwistWithCovarianceConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::TwistWithCovariance();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_twist_with_covariance(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsTwistWithCovarianceConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Pose = StaticCastSharedPtr<ROSMessages::geometry_msgs::TwistWithCovariance>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_twist_with_covariance(*message, Pose.Get());

	return true;
}
