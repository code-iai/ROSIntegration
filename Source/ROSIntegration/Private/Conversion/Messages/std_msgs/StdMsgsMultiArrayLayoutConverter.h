#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/MultiArrayLayout.h"
#include "Conversion/Messages/std_msgs/StdMsgsMultiArrayDimensionConverter.h"
#include "StdMsgsMultiArrayLayoutConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsMultiArrayLayoutConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsMultiArrayLayoutConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_multi_array_layout(bson_t *b, FString key, ROSMessages::std_msgs::MultiArrayLayout *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->dim = GetTArrayFromBSON<ROSMessages::std_msgs::MultiArrayDimension>(key + ".dim", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::std_msgs::MultiArrayDimension ret;
			subKeyFound = UStdMsgsMultiArrayDimensionConverter::_bson_extract_child_multi_array_dimension(subMsg, subKey, &ret, LogOnErrors);
			return ret;
		}, LogOnErrors);
		if (!KeyFound) return false;

		msg->data_offset = GetInt32FromBSON(key + ".data_offset", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_multi_array_layout(bson_t *b, const char *key, const ROSMessages::std_msgs::MultiArrayLayout *msg)
	{
		bson_t layout;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &layout);
		_bson_append_multi_array_layout(&layout, msg);
		bson_append_document_end(b, &layout);
	}

	static void _bson_append_multi_array_layout(bson_t *b, const ROSMessages::std_msgs::MultiArrayLayout *msg)
	{
		_bson_append_tarray<ROSMessages::std_msgs::MultiArrayDimension>(b, "dim", msg->dim, [](bson_t *subb, const char *subKey, ROSMessages::std_msgs::MultiArrayDimension mad)
		{
			UStdMsgsMultiArrayDimensionConverter::_bson_append_child_multi_array_dimension(subb, subKey, &mad);
		});
		BSON_APPEND_INT32(b, "data_offset", msg->data_offset);
	}
};