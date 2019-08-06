#include "NavMsgsPathConverter.h"

#include "nav_msgs/Path.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseStampedConverter.h"


UNavMsgsPathConverter::UNavMsgsPathConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "nav_msgs/Path";
}

bool UNavMsgsPathConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::nav_msgs::Path();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);

	bool KeyFound = false;
	bson_t *b = message->full_msg_bson_;

	KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, TEXT(".header"), &p->header); if (!KeyFound) return false;

	p->poses = GetTArrayFromBSON<ROSMessages::geometry_msgs::PoseStamped>(FString(".poses"), b, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) {
		ROSMessages::geometry_msgs::PoseStamped ret;
		subKeyFound = UGeometryMsgsPoseStampedConverter::_bson_extract_child_pose_stamped(subMsg, subKey, &ret);
		return ret;
	});
	if (!KeyFound) return false;

	return true;
}

bool UNavMsgsPathConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Path = StaticCastSharedPtr<ROSMessages::nav_msgs::Path>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);

	UStdMsgsHeaderConverter::_bson_append_header(*message, &(Path->header));
	_bson_append_tarray<ROSMessages::geometry_msgs::PoseStamped>(*message, "poses", Path->poses, [](bson_t* msg, const char* key, const ROSMessages::geometry_msgs::PoseStamped& pose_stamped)
	{
		UGeometryMsgsPoseStampedConverter::_bson_append_child_pose_stamped(msg, key, &pose_stamped);
	});

	return true;
}
