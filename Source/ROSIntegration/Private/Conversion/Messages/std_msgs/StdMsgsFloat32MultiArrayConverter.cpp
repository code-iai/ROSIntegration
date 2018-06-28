#include "Conversion/Messages/std_msgs/StdMsgsFloat32MultiArrayConverter.h"

UStdMsgsFloat32MultiArrayConverter::UStdMsgsFloat32MultiArrayConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/Float32MultiArray";
}

bool UStdMsgsFloat32MultiArrayConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::std_msgs::Float32MultiArray;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_float_multi_array(message->full_msg_bson_, "msg", p);
}

bool UStdMsgsFloat32MultiArrayConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto mad = StaticCastSharedPtr<ROSMessages::std_msgs::Float32MultiArray>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_float_multi_array(*message, mad.Get());

	return true;
}
