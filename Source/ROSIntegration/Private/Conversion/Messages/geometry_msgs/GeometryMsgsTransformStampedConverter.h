#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "geometry_msgs/TransformStamped.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTransformConverter.h"

#include "GeometryMsgsTransformStampedConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTransformStampedConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

    static bool _bson_extract_child_transform_stamped(bson_t *b, FString key, ROSMessages::geometry_msgs::TransformStamped *p, bool LogOnErrors = true)
    {
        bool keyFound = false;
        if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &(p->header), LogOnErrors)) return false;
        p->child_frame_id = GetFStringFromBSON(key + ".child_frame_id", b, keyFound, LogOnErrors); if (!keyFound) return false;
        if (!UGeometryMsgsTransformConverter::_bson_extract_child_transform(b, key + ".transform", &(p->transform), LogOnErrors)) return false;
        return true;
    }
};
