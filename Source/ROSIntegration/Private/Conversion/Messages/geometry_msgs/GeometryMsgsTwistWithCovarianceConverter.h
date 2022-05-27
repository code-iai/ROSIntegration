#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistConverter.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "GeometryMsgsTwistWithCovarianceConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTwistWithCovarianceConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsTwistWithCovarianceConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_twist_with_covariance(bson_t *b, FString key, ROSMessages::geometry_msgs::TwistWithCovariance *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		if (!UGeometryMsgsTwistConverter::_bson_extract_child_twist(b, key + ".twist", &msg->twist, LogOnErrors))
			return false;

		msg->covariance = GetDoubleTArrayFromBSON(key + ".covariance", b, KeyFound, LogOnErrors);
		if (!KeyFound || msg->covariance.Num() != 36) // ROS requires there to be 36 elements
			return false;

		return true;
	}

	static void _bson_append_child_twist_with_covariance(bson_t *b, const char *key, const ROSMessages::geometry_msgs::TwistWithCovariance *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_twist_with_covariance(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_twist_with_covariance(bson_t *b, const ROSMessages::geometry_msgs::TwistWithCovariance *msg)
	{
		UGeometryMsgsTwistConverter::_bson_append_child_twist(b, "twist", &msg->twist);
		_bson_append_double_tarray(b, "covariance", msg->covariance);
	}
};