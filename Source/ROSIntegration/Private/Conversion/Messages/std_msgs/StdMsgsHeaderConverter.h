#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Header.h"
#include "StdMsgsHeaderConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsHeaderConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsHeaderConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_header(bson_t *b, FString key, ROSMessages::std_msgs::Header *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		
		msg->seq = 0;
		if (ROS_VERSION == 1)
		{
			msg->seq = GetInt32FromBSON(key + ".seq", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		}

		if (!_bson_extract_child_ros_time(b, key + ".stamp", &msg->time, LogOnErrors)) return false;
		msg->frame_id = GetFStringFromBSON(key + ".frame_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_header(bson_t* b, const char* key, const ROSMessages::std_msgs::Header* msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_header(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_header(bson_t *b, const ROSMessages::std_msgs::Header *msg)
	{
		if (ROS_VERSION == 1)
			BSON_APPEND_INT32(b, "seq", msg->seq);

		_bson_append_child_ros_time(b, "stamp", &msg->time);

		BSON_APPEND_UTF8(b, "frame_id", TCHAR_TO_UTF8(*msg->frame_id));
	}
};
