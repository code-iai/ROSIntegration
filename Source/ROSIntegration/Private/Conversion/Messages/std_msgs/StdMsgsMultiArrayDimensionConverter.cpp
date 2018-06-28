#include "Conversion/Messages/std_msgs/StdMsgsMultiArrayDimensionConverter.h"

UStdMsgsMultiArrayDimensionConverter::UStdMsgsMultiArrayDimensionConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/MultiArrayDimension";
}

bool UStdMsgsMultiArrayDimensionConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::std_msgs::MultiArrayDimension;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_multi_array_dimension(message->full_msg_bson_, "msg", p);
}

bool UStdMsgsMultiArrayDimensionConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto mad = StaticCastSharedPtr<ROSMessages::std_msgs::MultiArrayDimension>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_multi_array_dimension(*message, mad.Get());

	return true;
}
