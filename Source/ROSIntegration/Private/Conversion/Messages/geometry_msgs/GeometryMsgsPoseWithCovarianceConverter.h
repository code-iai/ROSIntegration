#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "GeometryMsgsPoseWithCovarianceConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPoseWithCovarianceConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsPoseWithCovarianceConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_pose_with_covariance(bson_t *b, FString key, ROSMessages::geometry_msgs::PoseWithCovariance *p, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".pose", &p->pose, LogOnErrors))
			return false;

		p->covariance = GetDoubleTArrayFromBSON(key + ".covariance", b, KeyFound, LogOnErrors);
		if (!KeyFound || p->covariance.Num() != 36) // ROS requires there to be 36 elements
			return false;

		return true;
	}

	static void _bson_append_child_pose_with_covariance(bson_t *b, const char *key, const ROSMessages::geometry_msgs::PoseWithCovariance *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_pose_with_covariance(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_pose_with_covariance(bson_t *b, const ROSMessages::geometry_msgs::PoseWithCovariance *msg)
	{
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "pose", &msg->pose);
		_bson_append_double_tarray(b, "covariance", msg->covariance);
	}
};