#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalIDConverter.h"
#include "actionlib_msgs/GoalStatus.h"
#include "ActionlibMsgsGoalStatusConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UActionlibMsgsGoalStatusConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_goal_status(bson_t *b, FString key, ROSMessages::actionlib_msgs::GoalStatus *g, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		KeyFound = UActionlibMsgsGoalIDConverter::_bson_extract_child_goal_id(b, key + ".goal_id", &g->goal_id, LogOnErrors); if (!KeyFound) return false;
		g->status = static_cast<ROSMessages::actionlib_msgs::GoalStatus::Status>( GetInt32FromBSON(key + ".status", b, KeyFound, LogOnErrors) );  if (!KeyFound) return false; // TODO: there is no GetUInt8FromBSON, do we need to implement that?
		g->text = GetFStringFromBSON(key + ".text", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_goal_status(bson_t *b, const char *key, ROSMessages::actionlib_msgs::GoalStatus *g)
	{
		bson_t goalStatus;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &goalStatus);
		_bson_append_goal_status(&goalStatus, g);
		bson_append_document_end(b, &goalStatus);
	}

	static void _bson_append_goal_status(bson_t *b, ROSMessages::actionlib_msgs::GoalStatus *g)
	{
		UActionlibMsgsGoalIDConverter::_bson_append_child_goal_id(b, "goal_id", &(g->goal_id));
		BSON_APPEND_INT32(b, "status", g->status);
		BSON_APPEND_UTF8(b, "text", TCHAR_TO_UTF8(*g->text));
	}
};
