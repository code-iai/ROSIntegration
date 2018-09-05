#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalStatusConverter.h"


UActionlibMsgsGoalStatusConverter::UActionlibMsgsGoalStatusConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "actionlib_msgs/GoalStatus";
}

bool UActionlibMsgsGoalStatusConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto g = new ROSMessages::actionlib_msgs::GoalStatus();
	BaseMsg = TSharedPtr<FROSBaseMsg>(g);
	return _bson_extract_child_goal_status(message->full_msg_bson_, "msg", g);
}

bool UActionlibMsgsGoalStatusConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto GoalStatus = StaticCastSharedPtr<ROSMessages::actionlib_msgs::GoalStatus>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_goal_status(*message, GoalStatus.Get());

	return true;
}
