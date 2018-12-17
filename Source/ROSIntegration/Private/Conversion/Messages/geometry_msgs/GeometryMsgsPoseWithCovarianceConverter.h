#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"

#include "GeometryMsgsPoseWithCovarianceConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPoseWithCovarianceConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_pose_with_covariance(bson_t *b, FString key, ROSMessages::geometry_msgs::PoseWithCovariance *p, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".pose", &p->pose, LogOnErrors))
			return false;

		p->covariance = GetDoubleTArrayFromBSON(key + ".covariance", b, KeyFound, LogOnErrors);
		if (!KeyFound || p->covariance.Num() != 36) // TODO: Magic Number?
			return false;

		return true;
	}

	static void _bson_append_child_pose_with_covariance(bson_t *b, const char *key, ROSMessages::geometry_msgs::PoseWithCovariance *t)
	{
		bson_t pose;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &pose);
		_bson_append_pose_with_covariance(&pose, t);
		bson_append_document_end(b, &pose);
	}

	static void _bson_append_pose_with_covariance(bson_t *b, ROSMessages::geometry_msgs::PoseWithCovariance *t)
	{
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "pose", &(t->pose));
		_bson_append_double_tarray(b, "covariance", t->covariance);
	}
};
