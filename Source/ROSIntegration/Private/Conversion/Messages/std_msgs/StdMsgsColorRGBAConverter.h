#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/ColorRGBA.h"
#include "StdMsgsColorRGBAConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsColorRGBAConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsColorRGBAConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_color_rgba(bson_t *b, FString key, ROSMessages::std_msgs::ColorRGBA *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		msg->r = GetDoubleFromBSON(key + ".r", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->g = GetDoubleFromBSON(key + ".g", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->b = GetDoubleFromBSON(key + ".b", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->a = GetDoubleFromBSON(key + ".a", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_color_rgba(bson_t* b, const char* key, const ROSMessages::std_msgs::ColorRGBA* msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_color_rgba(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_color_rgba(bson_t *b, const ROSMessages::std_msgs::ColorRGBA *msg)
	{
		BSON_APPEND_DOUBLE(b, "r", msg->r);
		BSON_APPEND_DOUBLE(b, "g", msg->g);
		BSON_APPEND_DOUBLE(b, "b", msg->b);
		BSON_APPEND_DOUBLE(b, "a", msg->a);
	}
};
