#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "rosgraph_msgs/Clock.h"
#include "ROSGraphMsgsClockConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UROSGraphMsgsClockConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UROSGraphMsgsClockConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_clock(bson_t *b, FString key, ROSMessages::rosgraph_msgs::Clock *msg, bool LogOnErrors = true)
	{
		return _bson_extract_child_ros_time(b, key + ".clock", &msg->_Clock, LogOnErrors);
	}

	static void _bson_append_child_clock(bson_t *b, const char *key, const ROSMessages::rosgraph_msgs::Clock *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_clock(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_clock(bson_t *b, const ROSMessages::rosgraph_msgs::Clock *msg)
	{
		_bson_append_child_ros_time(b, "clock", &msg->_Clock);
	}
};
