#pragma once

#include <CoreMinimal.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Point.h"
#include "GeometryMsgsPointConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPointConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsPointConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_point(bson_t *b, FString key, ROSMessages::geometry_msgs::Point *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->x = GetDoubleFromBSON(key + ".x", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->y = GetDoubleFromBSON(key + ".y", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->z = GetDoubleFromBSON(key + ".z", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}


	static void _bson_append_child_point(bson_t *b, const char *key, const ROSMessages::geometry_msgs::Point *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_point(&child, msg);
		bson_append_document_end(b, &child);
	}


	static void _bson_append_point(bson_t *b, const ROSMessages::geometry_msgs::Point *msg)
	{
		BSON_APPEND_DOUBLE(b, "x", msg->x);
		BSON_APPEND_DOUBLE(b, "y", msg->y);
		BSON_APPEND_DOUBLE(b, "z", msg->z);
	}
};
