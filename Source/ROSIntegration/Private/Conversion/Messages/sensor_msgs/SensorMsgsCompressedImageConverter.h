#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "sensor_msgs/CompressedImage.h"
#include "SensorMsgsCompressedImageConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsCompressedImageConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsCompressedImageConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_image(bson_t *b, FString key, ROSMessages::sensor_msgs::CompressedImage *msg)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->format = GetFStringFromBSON(key + ".format", b, KeyFound); if (!KeyFound) return false;
		msg->data = GetBinaryFromBSON(key + ".data", b, KeyFound); if (!KeyFound) return false;

		return KeyFound;
	}

	static void _bson_append_child_image(bson_t *b, const char *key, const ROSMessages::sensor_msgs::CompressedImage *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_image(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_image(bson_t *b, const ROSMessages::sensor_msgs::CompressedImage *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_UTF8(b, "format", TCHAR_TO_UTF8(*msg->format));
		BSON_APPEND_BINARY(b, "data", BSON_SUBTYPE_BINARY, msg->data, msg->data_size);
	}
};