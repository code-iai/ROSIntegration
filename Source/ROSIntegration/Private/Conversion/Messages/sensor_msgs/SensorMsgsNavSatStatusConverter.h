#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "sensor_msgs/NavSatStatus.h"
#include "Conversion/Messages/BaseMessageConverter.h"

#include "SensorMsgsNavSatStatusConverter.generated.h"

UCLASS()
class ROSINTEGRATION_API USensorMsgsNavSatStatusConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_nav_sat_status(bson_t* b, FString key, ROSMessages::sensor_msgs::NavSatStatus* nss, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		nss->status = static_cast<ROSMessages::sensor_msgs::NavSatStatus::Status>(GetInt32FromBSON(key + ".status", b, KeyFound, LogOnErrors));
		if (!KeyFound) return false;

		nss->service = static_cast<ROSMessages::sensor_msgs::NavSatStatus::Service>(GetInt32FromBSON(key + ".service", b, KeyFound, LogOnErrors));
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_nav_sat_status(bson_t* b, const char *key, ROSMessages::sensor_msgs::NavSatStatus* nss)
	{
		bson_t result;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &result);
		_bson_append_nav_sat_status(&result, nss);
		bson_append_document_end(b, &result);
	}

	static void _bson_append_nav_sat_status(bson_t* b, ROSMessages::sensor_msgs::NavSatStatus* nss)
	{
		BSON_APPEND_INT32(b, "status", nss->status);
		BSON_APPEND_INT32(b, "service", nss->service);
	}
};
