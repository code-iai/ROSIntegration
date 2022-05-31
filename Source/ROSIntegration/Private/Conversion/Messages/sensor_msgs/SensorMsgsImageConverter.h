#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "sensor_msgs/Image.h"
#include "SensorMsgsImageConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsImageConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsImageConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_image(bson_t *b, FString key, ROSMessages::sensor_msgs::Image *msg)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->height = GetInt32FromBSON(key + ".height", b, KeyFound); if (!KeyFound) return false;
		msg->width = GetInt32FromBSON(key + ".width", b, KeyFound); if (!KeyFound) return false;
		msg->encoding = GetFStringFromBSON(key + ".encoding", b, KeyFound); if (!KeyFound) return false;
		msg->is_bigendian = GetInt32FromBSON(key + ".is_bigendian", b, KeyFound); if (!KeyFound) return false;
		msg->step = GetInt32FromBSON(key + ".step", b, KeyFound); if (!KeyFound) return false;
		msg->data = GetBinaryFromBSON(key + ".data", b, KeyFound); if (!KeyFound) return false;

		return KeyFound;
	}

	static void _bson_append_child_image(bson_t *b, const char *key, const ROSMessages::sensor_msgs::Image *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_image(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_image(bson_t *b, const ROSMessages::sensor_msgs::Image *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_INT32(b, "height", msg->height);
		BSON_APPEND_INT32(b, "width", msg->width);
		BSON_APPEND_UTF8(b, "encoding", TCHAR_TO_UTF8(*msg->encoding));
		BSON_APPEND_INT32(b, "is_bigendian", msg->is_bigendian);
		BSON_APPEND_INT32(b, "step", msg->step);
		BSON_APPEND_BINARY(b, "data", BSON_SUBTYPE_BINARY, msg->data, msg->height * msg->step);
	}
};