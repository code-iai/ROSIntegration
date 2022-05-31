#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Float32MultiArray.h"
#include "Conversion/Messages/std_msgs/StdMsgsMultiArrayLayoutConverter.h"
#include "StdMsgsFloat32MultiArrayConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsFloat32MultiArrayConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsFloat32MultiArrayConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_float_multi_array(bson_t *b, FString key, ROSMessages::std_msgs::Float32MultiArray *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsMultiArrayLayoutConverter::_bson_extract_child_multi_array_layout(b, key + ".layout", &msg->layout, LogOnErrors); if (!KeyFound) return false;
		msg->data = GetFloatTArrayFromBSON(key + ".data", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_float_multi_array(bson_t *b, const char *key, const ROSMessages::std_msgs::Float32MultiArray *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_float_multi_array(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_float_multi_array(bson_t *b, const ROSMessages::std_msgs::Float32MultiArray *msg)
	{
		UStdMsgsMultiArrayLayoutConverter::_bson_append_child_multi_array_layout(b, "layout", &msg->layout);
		_bson_append_float_tarray(b, "data", msg->data);
	}
};