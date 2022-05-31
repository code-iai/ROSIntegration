#include "Conversion/Messages/nav_msgs/NavMsgsOdometryConverter.h"


UNavMsgsOdometryConverter::UNavMsgsOdometryConverter()
{
	_MessageType = "nav_msgs/Odometry";
}

bool UNavMsgsOdometryConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::nav_msgs::Odometry();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_odometry(message->full_msg_bson_, "msg", msg);
}

bool UNavMsgsOdometryConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::nav_msgs::Odometry>(BaseMsg);
	*message = bson_new();
	_bson_append_odometry(*message, CastMsg.Get());
	return true;
}