#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalStatusArrayConverter.h"


UActionlibMsgsGoalStatusArrayConverter::UActionlibMsgsGoalStatusArrayConverter()
{
	_MessageType = "actionlib_msgs/GoalStatusArray";
}

bool UActionlibMsgsGoalStatusArrayConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::actionlib_msgs::GoalStatusArray();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_goal_status_array(message->full_msg_bson_, "msg", msg);
}

bool UActionlibMsgsGoalStatusArrayConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMSG = StaticCastSharedPtr<ROSMessages::actionlib_msgs::GoalStatusArray>(BaseMsg);
	*message = bson_new();
	_bson_append_goal_status_array(*message, CastMSG.Get());
	return true;
}
