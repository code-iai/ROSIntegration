#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "actionlib_msgs/GoalID.h"
#include "ActionlibMsgsGoalIDConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UActionlibMsgsGoalIDConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UActionlibMsgsGoalIDConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_goal_id(bson_t *b, FString key, ROSMessages::actionlib_msgs::GoalID *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		int32 Sec =   GetInt32FromBSON(key + ".stamp.secs", b, KeyFound, LogOnErrors);  if (!KeyFound) return false;
		int32 NSec =  GetInt32FromBSON(key + ".stamp.nsecs", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->id = GetFStringFromBSON(key + ".id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->stamp = FROSTime(Sec, NSec);

		return true;
	}

	static void _bson_append_child_goal_id(bson_t *b, const char *key, const ROSMessages::actionlib_msgs::GoalID *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_goal_id(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_goal_id(bson_t *b, const ROSMessages::actionlib_msgs::GoalID *msg)
	{
		bson_t stamp;
		BSON_APPEND_DOCUMENT_BEGIN(b, "stamp", &stamp);
		BSON_APPEND_INT32(&stamp, "secs", msg->stamp._Sec);
		BSON_APPEND_INT32(&stamp, "nsecs", msg->stamp._NSec);
		bson_append_document_end(b, &stamp);
		BSON_APPEND_UTF8(b, "id", TCHAR_TO_UTF8(*msg->id));
	}
};
