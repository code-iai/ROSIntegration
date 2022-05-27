#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalIDConverter.h"


UActionlibMsgsGoalIDConverter::UActionlibMsgsGoalIDConverter()
{
	_MessageType = "actionlib_msgs/GoalID";
}

bool UActionlibMsgsGoalIDConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::actionlib_msgs::GoalID();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_goal_id(message->full_msg_bson_, "msg", msg);
}

bool UActionlibMsgsGoalIDConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMSG = StaticCastSharedPtr<ROSMessages::actionlib_msgs::GoalID>(BaseMsg);
	*message = bson_new();
	_bson_append_goal_id(*message, CastMSG.Get());
	return true;
}
