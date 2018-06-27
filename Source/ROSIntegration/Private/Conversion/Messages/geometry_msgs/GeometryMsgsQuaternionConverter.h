#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Quaternion.h"

#include "GeometryMsgsQuaternionConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsQuaternionConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_quaternion(bson_t *b, FString key, ROSMessages::geometry_msgs::Quaternion *q)
	{
		bool KeyFound = false;

		q->x = GetDoubleFromBSON(key + ".x", b, KeyFound); if (!KeyFound) return false;
		q->y = GetDoubleFromBSON(key + ".y", b, KeyFound); if (!KeyFound) return false;
		q->z = GetDoubleFromBSON(key + ".z", b, KeyFound); if (!KeyFound) return false;
		q->w = GetDoubleFromBSON(key + ".w", b, KeyFound); if (!KeyFound) return false;
		return true;
	}


	static void _bson_append_child_quaternion(bson_t *b, const char *key, ROSMessages::geometry_msgs::Quaternion *q)
	{
		bson_t quat;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &quat);
		_bson_append_quaternion(&quat, q);
		bson_append_document_end(b, &quat);
	}


	static void _bson_append_quaternion(bson_t *b, ROSMessages::geometry_msgs::Quaternion *q)
	{
		BSON_APPEND_DOUBLE(b, "x", q->x);
		BSON_APPEND_DOUBLE(b, "y", q->x);
		BSON_APPEND_DOUBLE(b, "z", q->x);
		BSON_APPEND_DOUBLE(b, "w", q->w);
	}
};
