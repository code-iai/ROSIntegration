#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Twist.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"

#include "GeometryMsgsTwistConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTwistConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_twist(bson_t *b, FString key, ROSMessages::geometry_msgs::Twist *p, bool LogOnErrors = true)
	{
		if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".linear", &p->linear, LogOnErrors)) return false;
		if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".angular", &p->angular, LogOnErrors)) return false;

		return true;
	}

	static void _bson_append_child_twist(bson_t *b, const char *key, ROSMessages::geometry_msgs::Twist *t)
	{
		bson_t twist;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &twist);
		_bson_append_twist(&twist, t);
		bson_append_document_end(b, &twist);
	}

	static void _bson_append_twist(bson_t *b, ROSMessages::geometry_msgs::Twist *t)
	{
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "linear", &(t->linear));
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "angular", &(t->angular));
	}
};
