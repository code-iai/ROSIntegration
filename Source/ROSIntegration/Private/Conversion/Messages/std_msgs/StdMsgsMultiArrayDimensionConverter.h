#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/MultiArrayDimension.h"

#include "StdMsgsMultiArrayDimensionConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsMultiArrayDimensionConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_multi_array_dimension(bson_t *b, FString key, ROSMessages::std_msgs::MultiArrayDimension *mad, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		mad->label = GetFStringFromBSON(key + ".label", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		mad->size = GetInt32FromBSON(key + ".size", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		mad->stride = GetInt32FromBSON(key + ".stride", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_multi_array_dimension(bson_t *b, const char *key, ROSMessages::std_msgs::MultiArrayDimension *mad)
	{
		bson_t m;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &m);
		_bson_append_multi_array_dimension(&m, mad);
		bson_append_document_end(b, &m);
	}


	static void _bson_append_multi_array_dimension(bson_t *b, ROSMessages::std_msgs::MultiArrayDimension *mad)
	{
		BSON_APPEND_UTF8(b, "label", TCHAR_TO_UTF8(*mad->label));
		BSON_APPEND_INT32(b, "size", mad->size);
		BSON_APPEND_INT32(b, "stride", mad->stride);
	}
};
