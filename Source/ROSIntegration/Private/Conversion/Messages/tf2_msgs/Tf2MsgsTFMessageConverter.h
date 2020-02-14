#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformStampedConverter.h"

#include "Tf2MsgsTFMessageConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UTf2MsgsTFMessageConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

    static ROSMessages::geometry_msgs::TransformStamped GetTransformStampedFromBSON(FString key, bson_t* msg, bool &keyFound, bool LogOnErrors = true)
    {
        auto p = new ROSMessages::geometry_msgs::TransformStamped;
        keyFound = UGeometryMsgsTransformStampedConverter::_bson_extract_child_transform_stamped(msg, key, p, LogOnErrors);
        if (!keyFound && LogOnErrors) {
            UE_LOG(LogTemp, Error, TEXT("Key %s is not present in data"), *key);
        }
        return *p;
    }

    static TArray<ROSMessages::geometry_msgs::TransformStamped> GetTFMessageTArrayFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
    {
        return GetTArrayFromBSON<ROSMessages::geometry_msgs::TransformStamped>(
                Key, msg, KeyFound, 
                [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetTransformStampedFromBSON(subKey, subMsg, subKeyFound, LogOnErrors); },
            LogOnErrors);
    }
};
