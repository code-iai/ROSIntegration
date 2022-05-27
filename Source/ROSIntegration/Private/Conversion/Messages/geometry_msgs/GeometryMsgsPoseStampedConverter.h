#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/PoseStamped.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"
#include "GeometryMsgsPoseStampedConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPoseStampedConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGeometryMsgsPoseStampedConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_pose_stamped(bson_t *b, FString key, ROSMessages::geometry_msgs::PoseStamped * msg, bool LogOnErrors = true)
	{
		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".pose", &msg->pose)) return false;
		return true;
	}

	static void _bson_append_child_pose_stamped(bson_t *b, const char *key, const ROSMessages::geometry_msgs::PoseStamped * msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_pose_stamped(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_pose_stamped(bson_t *b, const ROSMessages::geometry_msgs::PoseStamped * msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "pose", &msg->pose);
	}
};