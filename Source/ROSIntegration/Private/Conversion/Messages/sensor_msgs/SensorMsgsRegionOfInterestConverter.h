#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "SensorMsgsRegionOfInterestConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsRegionOfInterestConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsRegionOfInterestConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_roi(bson_t *b, FString key, ROSMessages::sensor_msgs::RegionOfInterest *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->x_offset = GetDoubleFromBSON(key + ".x_offset", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->y_offset = GetDoubleFromBSON(key + ".y_offset", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->height = GetDoubleFromBSON(key + ".height", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->width = GetDoubleFromBSON(key + ".width", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->do_rectify = GetBoolFromBSON(key + ".do_rectify", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		return true;
	}

	static void _bson_append_child_roi(bson_t *b, const char *key, const ROSMessages::sensor_msgs::RegionOfInterest *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_roi(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_roi(bson_t *b, const ROSMessages::sensor_msgs::RegionOfInterest *msg)
	{
		BSON_APPEND_INT32(b, "x_offset", msg->x_offset);
		BSON_APPEND_INT32(b, "y_offset", msg->y_offset);
		BSON_APPEND_INT32(b, "height", msg->height);
		BSON_APPEND_INT32(b, "width", msg->width);
		BSON_APPEND_BOOL(b, "do_rectify", msg->do_rectify);
	}
};