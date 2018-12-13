#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"


UGeometryMsgsQuaternionConverter::UGeometryMsgsQuaternionConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Quaternion";
}

bool UGeometryMsgsQuaternionConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::Quaternion();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_quaternion(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsQuaternionConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Quaternion = StaticCastSharedPtr<ROSMessages::geometry_msgs::Quaternion>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_quaternion(*message, Quaternion.Get());

	return true;
}
