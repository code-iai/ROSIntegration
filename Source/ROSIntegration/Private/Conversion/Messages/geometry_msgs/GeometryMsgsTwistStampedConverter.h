#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistConverter.h"
#include "geometry_msgs/TwistStamped.h"
#include "GeometryMsgsTwistStampedConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTwistStampedConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsTwistStampedConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_twist_stamped(bson_t *b, FString key, ROSMessages::geometry_msgs::TwistStamped *msg, bool LogOnErrors = true)
	{
		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		if (!UGeometryMsgsTwistConverter::_bson_extract_child_twist(b, key + ".twist", &msg->twist, LogOnErrors)) return false;

		return true;
	}

	static void _bson_append_child_twist_stamped(bson_t *b, const char *key, const ROSMessages::geometry_msgs::TwistStamped *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_twist_stamped(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_twist_stamped(bson_t *b, const ROSMessages::geometry_msgs::TwistStamped *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		UGeometryMsgsTwistConverter::_bson_append_child_twist(b, "twist", &msg->twist);
	}
};