#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Vector3.h"
#include "GeometryMsgsVector3Converter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsVector3Converter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsVector3Converter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_vector3(bson_t *b, FString key, ROSMessages::geometry_msgs::Vector3 *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		msg->x = GetDoubleFromBSON(key + ".x", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->y = GetDoubleFromBSON(key + ".y", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->z = GetDoubleFromBSON(key + ".z", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_vector3(bson_t *b, const char *key, const ROSMessages::geometry_msgs::Vector3 *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_vector3(&child, msg);
		bson_append_document_end(b, &child);
	}


	static void _bson_append_vector3(bson_t *b, const ROSMessages::geometry_msgs::Vector3 *msg)
	{
		BSON_APPEND_DOUBLE(b, "x", msg->x);
		BSON_APPEND_DOUBLE(b, "y", msg->y);
		BSON_APPEND_DOUBLE(b, "z", msg->z);
	}
};