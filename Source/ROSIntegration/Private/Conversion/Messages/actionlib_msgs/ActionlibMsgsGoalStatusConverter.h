#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalIDConverter.h"
#include "actionlib_msgs/GoalStatus.h"
#include "ActionlibMsgsGoalStatusConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UActionlibMsgsGoalStatusConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UActionlibMsgsGoalStatusConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_goal_status(bson_t *b, FString key, ROSMessages::actionlib_msgs::GoalStatus *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		if (!UActionlibMsgsGoalIDConverter::_bson_extract_child_goal_id(b, key + ".goal_id", &msg->goal_id, LogOnErrors)) return false;
		msg->status = static_cast<ROSMessages::actionlib_msgs::GoalStatus::Status>( GetInt32FromBSON(key + ".status", b, KeyFound, LogOnErrors) );  if (!KeyFound) return false; // TODO: there is no GetUInt8FromBSON, do we need to implement that?
		msg->text = GetFStringFromBSON(key + ".text", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_goal_status(bson_t *b, const char *key, const ROSMessages::actionlib_msgs::GoalStatus *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_goal_status(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_goal_status(bson_t *b, const ROSMessages::actionlib_msgs::GoalStatus *msg)
	{
		UActionlibMsgsGoalIDConverter::_bson_append_child_goal_id(b, "goal_id", &msg->goal_id);
		BSON_APPEND_INT32(b, "status", msg->status);
		BSON_APPEND_UTF8(b, "text", TCHAR_TO_UTF8(*msg->text));
	}
};
