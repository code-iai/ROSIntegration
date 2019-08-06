#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "nav_msgs/Path.h"

#include "NavMsgsPathConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UNavMsgsPathConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_path(bson_t *b, FString key, ROSMessages::nav_msgs::Path * path, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, TEXT("msg.header"), &path->header);
		if (!KeyFound) return false;

		path->poses = GetTArrayFromBSON<ROSMessages::geometry_msgs::PoseStamped>(key + ".poses", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::geometry_msgs::PoseStamped ret;
			subKeyFound = UGeometryMsgsPoseStampedConverter::_bson_extract_child_pose_stamped(subMsg, subKey, &ret, LogOnErrors);
			return ret;
		}, LogOnErrors);
		if (!KeyFound) return false;

		return true;
	}
};
