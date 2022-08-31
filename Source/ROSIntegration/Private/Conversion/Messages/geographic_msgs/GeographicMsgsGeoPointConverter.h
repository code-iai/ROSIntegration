#pragma once

#include <CoreMinimal.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geographic_msgs/GeoPoint.h"
#include "Conversion/Messages/std_msgs/StdMsgsFloat32Converter.h"
#include "GeographicMsgsGeoPointConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeographicMsgsGeoPointConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeographicMsgsGeoPointConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_geo_point(bson_t *b, FString key, ROSMessages::geographic_msgs::GeoPoint * msg, bool LogOnErrors = true)
	{
		bool KeyFound = true;

		msg->latitude  = GetDoubleFromBSON(key + ".latitude",  b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->longitude = GetDoubleFromBSON(key + ".longitude", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->altitude  = GetDoubleFromBSON(key + ".altitude",  b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_geo_point(bson_t *b, const char *key, const ROSMessages::geographic_msgs::GeoPoint * msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_geo_point(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_geo_point(bson_t *b, const ROSMessages::geographic_msgs::GeoPoint * msg)
	{
		BSON_APPEND_DOUBLE(b, "latitude",  msg->latitude);
		BSON_APPEND_DOUBLE(b, "longitude", msg->longitude);
		BSON_APPEND_DOUBLE(b, "altitude",  msg->altitude);
	}
};
