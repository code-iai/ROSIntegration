#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/PoseStamped.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"

#include "GeometryMsgsPoseStampedConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsPoseStampedConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_pose_stamped(bson_t *b, FString key, ROSMessages::geometry_msgs::PoseStamped * ps, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &ps->header);
		if (!KeyFound) return false;

		KeyFound = UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".pose", &ps->pose);
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_pose_stamped(bson_t *b, const char *key, ROSMessages::geometry_msgs::PoseStamped * ps)
	{
		bson_t layout;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &layout);
		_bson_append_pose_stamped(&layout, ps);
		bson_append_document_end(b, &layout);
	}

	static void _bson_append_pose_stamped(bson_t *b, ROSMessages::geometry_msgs::PoseStamped * ps)
	{
		UStdMsgsHeaderConverter::_bson_append_header(b, &(ps->header));
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "pose", &(ps->pose));
	}
};
