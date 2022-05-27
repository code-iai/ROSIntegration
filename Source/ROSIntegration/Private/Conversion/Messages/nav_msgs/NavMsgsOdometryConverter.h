#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseWithCovarianceConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistWithCovarianceConverter.h"
#include "nav_msgs/Odometry.h"
#include "NavMsgsOdometryConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UNavMsgsOdometryConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UNavMsgsOdometryConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_odometry(bson_t *b, FString key, ROSMessages::nav_msgs::Odometry * msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		
		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->child_frame_id = GetFStringFromBSON(key + ".child_frame_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		if (!UGeometryMsgsPoseWithCovarianceConverter::_bson_extract_child_pose_with_covariance(b, key + ".pose", &msg->pose)) return false;
		if (!UGeometryMsgsTwistWithCovarianceConverter::_bson_extract_child_twist_with_covariance(b, key + ".twist", &msg->twist)) return false;
		
		return true;
	}

	static void _bson_append_child_odometry(bson_t *b, const char *key, const ROSMessages::nav_msgs::Odometry *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_odometry(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_odometry(bson_t *b, const ROSMessages::nav_msgs::Odometry *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_UTF8(b, "child_frame_id", TCHAR_TO_UTF8(*msg->child_frame_id));
		UGeometryMsgsPoseWithCovarianceConverter::_bson_append_child_pose_with_covariance(b, "pose", &msg->pose);
		UGeometryMsgsTwistWithCovarianceConverter::_bson_append_child_twist_with_covariance(b, "twist", &msg->twist);
	}
};