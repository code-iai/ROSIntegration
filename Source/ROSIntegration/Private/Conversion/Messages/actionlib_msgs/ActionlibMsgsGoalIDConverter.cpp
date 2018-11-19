#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalIDConverter.h"


UActionlibMsgsGoalIDConverter::UActionlibMsgsGoalIDConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "actionlib_msgs/GoalID";
}

bool UActionlibMsgsGoalIDConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto g = new ROSMessages::actionlib_msgs::GoalID();
	BaseMsg = TSharedPtr<FROSBaseMsg>(g);
	return _bson_extract_child_goal_id(message->full_msg_bson_, "msg", g);
}

bool UActionlibMsgsGoalIDConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto GoalID = StaticCastSharedPtr<ROSMessages::actionlib_msgs::GoalID>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_goal_id(*message, GoalID.Get());

	return true;
}
