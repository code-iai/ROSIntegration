#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"
#include "geometry_msgs/Twist.h"
#include "GeometryMsgsTwistConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTwistConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsTwistConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_twist(bson_t *b, FString key, ROSMessages::geometry_msgs::Twist *msg, bool LogOnErrors = true)
	{
		if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".linear", &msg->linear, LogOnErrors)) return false;
		if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".angular", &msg->angular, LogOnErrors)) return false;

		return true;
	}

	static void _bson_append_child_twist(bson_t *b, const char *key, const ROSMessages::geometry_msgs::Twist *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_twist(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_twist(bson_t *b, const ROSMessages::geometry_msgs::Twist *msg)
	{
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "linear", &msg->linear);
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "angular", &msg->angular);
	}
};