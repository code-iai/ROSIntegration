#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/actionlib_msgs/ActionlibMsgsGoalStatusConverter.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "ActionlibMsgsGoalStatusArrayConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UActionlibMsgsGoalStatusArrayConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UActionlibMsgsGoalStatusArrayConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_goal_status_array(bson_t *b, FString key, ROSMessages::actionlib_msgs::GoalStatusArray *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->status_list = GetTArrayFromBSON<ROSMessages::actionlib_msgs::GoalStatus>(key + ".status_list", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::actionlib_msgs::GoalStatus ret;
			subKeyFound = UActionlibMsgsGoalStatusConverter::_bson_extract_child_goal_status(subMsg, subKey, &ret, LogOnErrors);
			return ret;
		}, LogOnErrors);
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_goal_status_array(bson_t *b, const char *key, const ROSMessages::actionlib_msgs::GoalStatusArray *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_goal_status_array(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_goal_status_array(bson_t *b, const ROSMessages::actionlib_msgs::GoalStatusArray *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		_bson_append_tarray<ROSMessages::actionlib_msgs::GoalStatus>(b, "status_list", msg->status_list, [](bson_t *subb, const char *subKey, ROSMessages::actionlib_msgs::GoalStatus gs)
		{
			UActionlibMsgsGoalStatusConverter::_bson_append_child_goal_status(subb, subKey, &gs);
		});
	}
};