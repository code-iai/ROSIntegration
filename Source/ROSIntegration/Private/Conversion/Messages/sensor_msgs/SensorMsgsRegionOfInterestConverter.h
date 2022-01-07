#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "sensor_msgs/RegionOfInterest.h"
#include "Conversion/Messages/BaseMessageConverter.h"

#include "SensorMsgsRegionOfInterestConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsRegionOfInterestConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_roi(bson_t *b, FString key, ROSMessages::sensor_msgs::RegionOfInterest *roi, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		roi->x_offset = GetDoubleFromBSON(key + ".x_offset", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		roi->y_offset = GetDoubleFromBSON(key + ".y_offset", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		roi->height = GetDoubleFromBSON(key + ".height", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		roi->width = GetDoubleFromBSON(key + ".width", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		roi->do_rectify = GetBoolFromBSON(key + ".do_rectify", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_roi(bson_t *b, const char *key, const ROSMessages::sensor_msgs::RegionOfInterest *roi)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_roi(&child, roi);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_roi(bson_t *b, const ROSMessages::sensor_msgs::RegionOfInterest *roi)
	{
		BSON_APPEND_INT32(b, "x_offset", roi->x_offset);
		BSON_APPEND_INT32(b, "y_offset", roi->y_offset);
		BSON_APPEND_INT32(b, "height", roi->height);
		BSON_APPEND_INT32(b, "width", roi->width);
		BSON_APPEND_BOOL(b, "do_rectify", roi->do_rectify);
	}
};
