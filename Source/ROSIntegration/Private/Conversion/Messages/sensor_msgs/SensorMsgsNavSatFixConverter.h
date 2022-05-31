#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/sensor_msgs/SensorMsgsNavSatStatusConverter.h"
#include "sensor_msgs/NavSatFix.h"
#include "SensorMsgsNavSatFixConverter.generated.h"

UCLASS()
class ROSINTEGRATION_API USensorMsgsNavSatFixConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsNavSatFixConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_nav_sat_fix(bson_t *b, FString key, ROSMessages::sensor_msgs::NavSatFix *msg)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		if (!USensorMsgsNavSatStatusConverter::_bson_extract_child_nav_sat_status(b, key + ".status", &msg->status)) return false;

		msg->latitude = GetDoubleFromBSON(key + ".latitude", b, KeyFound); if (!KeyFound) return false;
		msg->longitude = GetDoubleFromBSON(key + ".longitude", b, KeyFound); if (!KeyFound) return false;
		msg->altitude = GetDoubleFromBSON(key + ".altitude", b, KeyFound); if (!KeyFound) return false;

		msg->position_covariance = GetDoubleTArrayFromBSON(key + ".position_covariance", b, KeyFound);
		if (!KeyFound || msg->position_covariance.Num() != 9) // Covariance is a 3x3 matrix, so we should have 9 elements.
			return false;

		msg->position_covariance_type = static_cast<ROSMessages::sensor_msgs::NavSatFix::CovarianceType>(GetInt32FromBSON(key + ".position_covariance_type", b, KeyFound));
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_nav_sat_fix(bson_t *b, const char *key, const ROSMessages::sensor_msgs::NavSatFix *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_nav_sat_fix(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_nav_sat_fix(bson_t *b, const ROSMessages::sensor_msgs::NavSatFix *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		USensorMsgsNavSatStatusConverter::_bson_append_child_nav_sat_status(b, "status", &msg->status);
		BSON_APPEND_DOUBLE(b, "latitude", msg->latitude);
		BSON_APPEND_DOUBLE(b, "longitude", msg->longitude);
		BSON_APPEND_DOUBLE(b, "altitude", msg->altitude);
		_bson_append_double_tarray(b, "position_covariance", msg->position_covariance);
		BSON_APPEND_INT32(b, "position_covariance_type", msg->position_covariance_type);
	}
};