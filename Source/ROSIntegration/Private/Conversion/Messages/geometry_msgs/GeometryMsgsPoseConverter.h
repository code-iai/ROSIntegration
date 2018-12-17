#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Pose.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"

#include "GeometryMsgsPoseConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPoseConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_pose(bson_t *b, FString key, ROSMessages::geometry_msgs::Pose *p, bool LogOnErrors = true)
	{
		if (!UGeometryMsgsPointConverter::_bson_extract_child_point(b, key + ".position", &p->position, LogOnErrors)) return false;
		if (!UGeometryMsgsQuaternionConverter::_bson_extract_child_quaternion(b, key + ".orientation", &p->orientation, LogOnErrors)) return false;

		return true;
	}

	static void _bson_append_child_pose(bson_t *b, const char *key, ROSMessages::geometry_msgs::Pose *t)
	{
		bson_t pose;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &pose);
		_bson_append_pose(&pose, t);
		bson_append_document_end(b, &pose);
	}

	static void _bson_append_pose(bson_t *b, ROSMessages::geometry_msgs::Pose *t)
	{
		UGeometryMsgsPointConverter::_bson_append_child_point(b, "position", &(t->position));
		UGeometryMsgsQuaternionConverter::_bson_append_child_quaternion(b, "orientation", &(t->orientation));
	}
};
