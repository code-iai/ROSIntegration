#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"

#include "geometry_msgs/Vector3.h"

UGeometryMsgsVector3Converter::UGeometryMsgsVector3Converter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "geometry_msgs/Vector3";
}

bool UGeometryMsgsVector3Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::geometry_msgs::Vector3();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_vector3(message->full_msg_bson_, "msg", p);
}

bool UGeometryMsgsVector3Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Vector3 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_vector3(*message, Vector3.Get());

	return true;
}
