#include "Conversion/Messages/std_msgs/StdMsgsFloat32MultiArrayConverter.h"

UStdMsgsFloat32MultiArrayConverter::UStdMsgsFloat32MultiArrayConverter()
{
	_MessageType = "std_msgs/Float32MultiArray";
}

bool UStdMsgsFloat32MultiArrayConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::std_msgs::Float32MultiArray;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_float_multi_array(message->full_msg_bson_, "msg", msg);
}

bool UStdMsgsFloat32MultiArrayConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::std_msgs::Float32MultiArray>(BaseMsg);
	*message = bson_new();
	_bson_append_float_multi_array(*message, CastMsg.Get());
	return true;
}