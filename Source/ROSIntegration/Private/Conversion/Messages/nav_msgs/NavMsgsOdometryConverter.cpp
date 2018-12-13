#include "NavMsgsOdometryConverter.h"

#include "nav_msgs/Odometry.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseWithCovarianceConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistWithCovarianceConverter.h"


UNavMsgsOdometryConverter::UNavMsgsOdometryConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "nav_msgs/Odometry";
}

bool UNavMsgsOdometryConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto o = new ROSMessages::nav_msgs::Odometry();
	BaseMsg = TSharedPtr<FROSBaseMsg>(o);

	bool KeyFound = false;
	KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(message->full_msg_bson_, TEXT("msg.header"), &o->header);
	if (!KeyFound) return false;

	o->child_frame_id = GetFStringFromBSON(TEXT("msg.child_frame_id"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	KeyFound = UGeometryMsgsPoseWithCovarianceConverter::_bson_extract_child_pose_with_covariance(message->full_msg_bson_, TEXT("msg.pose"), &o->pose);
	if (!KeyFound) return false;

	KeyFound = UGeometryMsgsTwistWithCovarianceConverter::_bson_extract_child_twist_with_covariance(message->full_msg_bson_, TEXT("msg.twist"), &o->twist);
	if (!KeyFound) return false;

	return true;
}

bool UNavMsgsOdometryConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Odometry = StaticCastSharedPtr<ROSMessages::nav_msgs::Odometry>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);

	UStdMsgsHeaderConverter::_bson_append_header(*message, &(Odometry->header));

	const char* id = TCHAR_TO_UTF8(*Odometry->child_frame_id);
	BSON_APPEND_UTF8(*message, "child_frame_id", id);

	UGeometryMsgsPoseWithCovarianceConverter::_bson_append_child_pose_with_covariance(*message, "pose", &(Odometry->pose));
	UGeometryMsgsTwistWithCovarianceConverter::_bson_append_child_twist_with_covariance(*message, "twist", &(Odometry->twist));

	return true;
}
