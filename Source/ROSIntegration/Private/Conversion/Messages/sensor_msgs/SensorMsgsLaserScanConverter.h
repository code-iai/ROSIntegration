#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "sensor_msgs/LaserScan.h"
#include "SensorMsgsLaserScanConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsLaserScanConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsLaserScanConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_laser_scan(bson_t *b, FString key, ROSMessages::sensor_msgs::LaserScan *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header); if (!KeyFound) return false;

		msg->angle_min		= GetDoubleFromBSON(key + ".angle_min", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->angle_max		= GetDoubleFromBSON(key + ".angle_max", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->angle_increment = GetDoubleFromBSON(key + ".angle_increment", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->time_increment	= GetDoubleFromBSON(key + ".time_increment", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->scan_time		= GetDoubleFromBSON(key + ".scan_time", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->range_min		= GetDoubleFromBSON(key + ".range_min", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->range_max		= GetDoubleFromBSON(key + ".range_max", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->ranges = GetFloatTArrayFromBSON(key + ".ranges", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->intensities = GetFloatTArrayFromBSON(key + ".intensities", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_laser_scan(bson_t *b, const char *key, const ROSMessages::sensor_msgs::LaserScan *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_laser_scan(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_laser_scan(bson_t *b, const ROSMessages::sensor_msgs::LaserScan *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_DOUBLE(b, "angle_min", msg->angle_min);
		BSON_APPEND_DOUBLE(b, "angle_max", msg->angle_max);
		BSON_APPEND_DOUBLE(b, "angle_increment", msg->angle_increment);
		BSON_APPEND_DOUBLE(b, "time_increment", msg->time_increment);
		BSON_APPEND_DOUBLE(b, "scan_time", msg->scan_time);
		BSON_APPEND_DOUBLE(b, "range_min", msg->range_min);
		BSON_APPEND_DOUBLE(b, "range_max", msg->range_max);
		_bson_append_float_tarray(b, "ranges", msg->ranges);
		_bson_append_float_tarray(b, "intensities", msg->intensities);
	}
};