#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformStampedConverter.h"
#include "tf2_msgs/TFMessage.h"
#include "Tf2MsgsTFMessageConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UTf2MsgsTFMessageConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
    UTf2MsgsTFMessageConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

    static ROSMessages::geometry_msgs::TransformStamped GetTransformStampedFromBSON(FString key, bson_t* b, bool &keyFound, bool LogOnErrors = true)
    {
        auto msg = new ROSMessages::geometry_msgs::TransformStamped;
        keyFound = UGeometryMsgsTransformStampedConverter::_bson_extract_child_transform_stamped(b, key, msg, LogOnErrors);
        if (!keyFound && LogOnErrors) {
            UE_LOG(LogTemp, Error, TEXT("Key %s is not present in data"), *key);
        }
        return *msg;
    }

    static TArray<ROSMessages::geometry_msgs::TransformStamped> GetTFMessageTArrayFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
    {
        return GetTArrayFromBSON<ROSMessages::geometry_msgs::TransformStamped>(
                Key, msg, KeyFound, 
                [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetTransformStampedFromBSON(subKey, subMsg, subKeyFound, LogOnErrors); },
            LogOnErrors);
    }

    static bool _bson_extract_child_tf2_msg(bson_t *b, FString key, ROSMessages::tf2_msgs::TFMessage *msg, bool LogOnErrors = true)
    {
        bool keyFound = false;
        msg->transforms = GetTFMessageTArrayFromBSON(key + ".transforms", b, keyFound, false);
        return keyFound;
    }

    static void _bson_append_child_tf2_msg(bson_t *b, const char *key, const ROSMessages::tf2_msgs::TFMessage *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_tf2_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

    static void _bson_append_tf2_msg(bson_t *b, const ROSMessages::tf2_msgs::TFMessage *msg)
    {
        _bson_append_tarray<ROSMessages::geometry_msgs::TransformStamped>(b, "transforms", msg->transforms, [](bson_t* msg, const char* key, const ROSMessages::geometry_msgs::TransformStamped& elem)
		{
			UGeometryMsgsTransformStampedConverter::_bson_append_child_transform_stamped(msg, key, &elem);
		});
    }
};