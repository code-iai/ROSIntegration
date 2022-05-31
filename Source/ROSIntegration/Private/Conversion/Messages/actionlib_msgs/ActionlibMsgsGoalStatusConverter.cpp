#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalStatusConverter.h"


UActionlibMsgsGoalStatusConverter::UActionlibMsgsGoalStatusConverter()
{
	_MessageType = "actionlib_msgs/GoalStatus";
}

bool UActionlibMsgsGoalStatusConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::actionlib_msgs::GoalStatus();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_goal_status(message->full_msg_bson_, "msg", msg);
}

bool UActionlibMsgsGoalStatusConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMSG = StaticCastSharedPtr<ROSMessages::actionlib_msgs::GoalStatus>(BaseMsg);
	*message = bson_new();
	_bson_append_goal_status(*message, CastMSG.Get());
	return true;
}
