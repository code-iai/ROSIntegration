#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"

#include "std_msgs/Header.h"


UStdMsgsHeaderConverter::UStdMsgsHeaderConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/Header";
}

bool UStdMsgsHeaderConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) 
{
	auto p = new ROSMessages::std_msgs::Header();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_header(message->full_msg_bson_, "msg", p);
}

bool UStdMsgsHeaderConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
	auto ConcreteMessage = StaticCastSharedPtr<ROSMessages::std_msgs::Header>(BaseMsg);
	_bson_append_header(*message, ConcreteMessage.Get());

	return true;
}