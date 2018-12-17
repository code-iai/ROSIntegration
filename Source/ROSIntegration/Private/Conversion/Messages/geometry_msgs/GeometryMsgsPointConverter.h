#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Point.h"

#include "GeometryMsgsPointConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPointConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_point(bson_t *b, FString key, ROSMessages::geometry_msgs::Point *p, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		p->x = GetDoubleFromBSON(key + ".x", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		p->y = GetDoubleFromBSON(key + ".y", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		p->z = GetDoubleFromBSON(key + ".z", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}


	static void _bson_append_child_point(bson_t *b, const char *key, ROSMessages::geometry_msgs::Point *p)
	{
		bson_t point;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &point);
		_bson_append_point(&point, p);
		bson_append_document_end(b, &point);
	}


	static void _bson_append_point(bson_t *b, ROSMessages::geometry_msgs::Point *p)
	{
		BSON_APPEND_DOUBLE(b, "x", p->x);
		BSON_APPEND_DOUBLE(b, "y", p->y);
		BSON_APPEND_DOUBLE(b, "z", p->z);
	}
};
