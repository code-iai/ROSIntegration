#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/MultiArrayDimension.h"
#include "StdMsgsMultiArrayDimensionConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsMultiArrayDimensionConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsMultiArrayDimensionConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_multi_array_dimension(bson_t *b, FString key, ROSMessages::std_msgs::MultiArrayDimension *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->label = GetFStringFromBSON(key + ".label", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->size = GetInt32FromBSON(key + ".size", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->stride = GetInt32FromBSON(key + ".stride", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_multi_array_dimension(bson_t *b, const char *key, const ROSMessages::std_msgs::MultiArrayDimension *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_multi_array_dimension(&child, msg);
		bson_append_document_end(b, &child);
	}


	static void _bson_append_multi_array_dimension(bson_t *b, const ROSMessages::std_msgs::MultiArrayDimension *msg)
	{
		BSON_APPEND_UTF8(b, "label", TCHAR_TO_UTF8(*msg->label));
		BSON_APPEND_INT32(b, "size", msg->size);
		BSON_APPEND_INT32(b, "stride", msg->stride);
	}
};