#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformConverter.h"
#include "geometry_msgs/TransformStamped.h"
#include "GeometryMsgsTransformStampedConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTransformStampedConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
    UGeometryMsgsTransformStampedConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

    static bool _bson_extract_child_transform_stamped(bson_t *b, FString key, ROSMessages::geometry_msgs::TransformStamped *msg, bool LogOnErrors = true)
    {
        bool keyFound = false;
        if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header, LogOnErrors)) return false;
        msg->child_frame_id = GetFStringFromBSON(key + ".child_frame_id", b, keyFound, LogOnErrors); if (!keyFound) return false;
        if (!UGeometryMsgsTransformConverter::_bson_extract_child_transform(b, key + ".transform", &msg->transform, LogOnErrors)) return false;
        return true;
    }

    static void _bson_append_child_transform_stamped(bson_t *b, const char *key, const ROSMessages::geometry_msgs::TransformStamped *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_transform_stamped(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_transform_stamped(bson_t *b, const ROSMessages::geometry_msgs::TransformStamped *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
        BSON_APPEND_UTF8(b, "child_frame_id", TCHAR_TO_UTF8(*msg->child_frame_id));
        UGeometryMsgsTransformConverter::_bson_append_child_transform(b, "transform", &msg->transform);
	}
};