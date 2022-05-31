#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "sensor_msgs/NavSatStatus.h"
#include "SensorMsgsNavSatStatusConverter.generated.h"

UCLASS()
class ROSINTEGRATION_API USensorMsgsNavSatStatusConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsNavSatStatusConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_nav_sat_status(bson_t* b, FString key, ROSMessages::sensor_msgs::NavSatStatus* msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->status = static_cast<ROSMessages::sensor_msgs::NavSatStatus::Status>(GetInt32FromBSON(key + ".status", b, KeyFound, LogOnErrors));
		if (!KeyFound) return false;

		msg->service = static_cast<ROSMessages::sensor_msgs::NavSatStatus::Service>(GetInt32FromBSON(key + ".service", b, KeyFound, LogOnErrors));
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_nav_sat_status(bson_t* b, const char *key, const ROSMessages::sensor_msgs::NavSatStatus* msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_nav_sat_status(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_nav_sat_status(bson_t* b, const ROSMessages::sensor_msgs::NavSatStatus* msg)
	{
		BSON_APPEND_INT32(b, "status", msg->status);
		BSON_APPEND_INT32(b, "service", msg->service);
	}
};