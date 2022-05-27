#pragma once

#include <CoreMinimal.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"
#include "geometry_msgs/Pose.h"
#include "GeometryMsgsPoseConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPoseConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsPoseConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_pose(bson_t *b, FString key, ROSMessages::geometry_msgs::Pose *msg, bool LogOnErrors = true)
	{
		if (!UGeometryMsgsPointConverter::_bson_extract_child_point(b, key + ".position", &msg->position, LogOnErrors)) return false;
		if (!UGeometryMsgsQuaternionConverter::_bson_extract_child_quaternion(b, key + ".orientation", &msg->orientation, LogOnErrors)) return false;

		return true;
	}

	static void _bson_append_child_pose(bson_t *b, const char *key, const ROSMessages::geometry_msgs::Pose *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_pose(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_pose(bson_t *b, const ROSMessages::geometry_msgs::Pose *msg)
	{
		UGeometryMsgsPointConverter::_bson_append_child_point(b, "position", &msg->position);
		UGeometryMsgsQuaternionConverter::_bson_append_child_quaternion(b, "orientation", &msg->orientation);
	}
};
