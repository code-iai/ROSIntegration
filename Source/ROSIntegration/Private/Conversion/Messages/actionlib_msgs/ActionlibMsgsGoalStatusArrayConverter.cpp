#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalStatusArrayConverter.h"


UActionlibMsgsGoalStatusArrayConverter::UActionlibMsgsGoalStatusArrayConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "actionlib_msgs/GoalStatusArray";
}

bool UActionlibMsgsGoalStatusArrayConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto g = new ROSMessages::actionlib_msgs::GoalStatusArray();
	BaseMsg = TSharedPtr<FROSBaseMsg>(g);
	return _bson_extract_child_goal_status_array(message->full_msg_bson_, "msg", g);
}

bool UActionlibMsgsGoalStatusArrayConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto GoalStatusArray = StaticCastSharedPtr<ROSMessages::actionlib_msgs::GoalStatusArray>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_goal_status_array(*message, GoalStatusArray.Get());

	return true;
}
