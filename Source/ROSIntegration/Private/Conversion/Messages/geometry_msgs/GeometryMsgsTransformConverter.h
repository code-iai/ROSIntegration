#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Transform.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"
#include "GeometryMsgsTransformConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTransformConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsTransformConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

    static bool _bson_extract_child_transform(bson_t *b, FString key, ROSMessages::geometry_msgs::Transform *msg, bool LogOnErrors = true)
    {
        if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".translation", &msg->translation)) return false;
        if (!UGeometryMsgsQuaternionConverter::_bson_extract_child_quaternion(b, key + ".rotation", &msg->rotation)) return false;
        return true;
    }
    
    static void _bson_append_child_transform(bson_t *b, const char *key, const ROSMessages::geometry_msgs::Transform *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_transform(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_transform(bson_t *b, const ROSMessages::geometry_msgs::Transform *msg)
	{
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "translation", &msg->translation);
		UGeometryMsgsQuaternionConverter::_bson_append_child_quaternion(b, "rotation", &msg->rotation);
	}
};