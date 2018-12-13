#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "sensor_msgs/LaserScan.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"

#include "SensorMsgsLaserScanConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsLaserScanConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_laser_scan(bson_t *b, FString key, ROSMessages::sensor_msgs::LaserScan *ls, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &ls->header); if (!KeyFound) return false;

		ls->angle_min		= (float)GetDoubleFromBSON(key + ".angle_min", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		ls->angle_max		= (float)GetDoubleFromBSON(key + ".angle_max", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		ls->angle_increment	= (float)GetDoubleFromBSON(key + ".angle_increment", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		ls->time_increment	= (float)GetDoubleFromBSON(key + ".time_increment", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		ls->scan_time		= (float)GetDoubleFromBSON(key + ".scan_time", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		ls->range_min		= (float)GetDoubleFromBSON(key + ".range_min", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		ls->range_max		= (float)GetDoubleFromBSON(key + ".range_max", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		ls->ranges = GetFloatTArrayFromBSON(key + ".ranges", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		ls->intensities = GetFloatTArrayFromBSON(key + ".intensities", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_laser_scan(bson_t *b, const char *key, ROSMessages::sensor_msgs::LaserScan *ls)
	{
		bson_t layout;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &layout);
		_bson_append_laser_scan(&layout, ls);
		bson_append_document_end(b, &layout);
	}

	static void _bson_append_laser_scan(bson_t *b, ROSMessages::sensor_msgs::LaserScan *ls)
	{
		UStdMsgsHeaderConverter::_bson_append_header(b, &ls->header);

		BSON_APPEND_DOUBLE(b, "angle_min", ls->angle_min);
		BSON_APPEND_DOUBLE(b, "angle_max", ls->angle_max);
		BSON_APPEND_DOUBLE(b, "angle_increment", ls->angle_increment);
		BSON_APPEND_DOUBLE(b, "time_increment", ls->time_increment);
		BSON_APPEND_DOUBLE(b, "scan_time", ls->scan_time);
		BSON_APPEND_DOUBLE(b, "range_min", ls->range_min);
		BSON_APPEND_DOUBLE(b, "range_max", ls->range_max);

		_bson_append_float_tarray(b, "ranges", ls->ranges);
		_bson_append_float_tarray(b, "intensities", ls->intensities);
	}
};
