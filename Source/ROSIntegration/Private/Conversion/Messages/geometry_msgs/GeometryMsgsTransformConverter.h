#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Transform.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"

#include "GeometryMsgsTransformConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTransformConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static void _bson_append_child_transform(bson_t *b, const char *key, ROSMessages::geometry_msgs::Transform *t)
	{
		bson_t tform;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &tform);
		_bson_append_transform(b, t);
		bson_append_document_end(b, &tform);
	}

	static void _bson_append_transform(bson_t *b, ROSMessages::geometry_msgs::Transform *t)
	{
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "translation", &(t->translation));
		UGeometryMsgsQuaternionConverter::_bson_append_child_quaternion(b, "rotation", &(t->rotation));
	}
};
