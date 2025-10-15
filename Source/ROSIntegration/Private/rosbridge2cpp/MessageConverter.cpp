#include "MessageConverter.h"

UMessageConverter::UMessageConverter()
{
	_MessageType = TEXT("std_msgs/msg/String");
}

bool UMessageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg>& BaseMsg)
{
	bool KeyFound = false;

	FString Data = GetFStringFromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::String(Data));
	return true;
}

bool UMessageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto StringMessage = StaticCastSharedPtr<ROSMessages::std_msgs::String>(BaseMsg);
	*message = BCON_NEW(
		"data", TCHAR_TO_UTF8(*StringMessage->_Data)
	);
	return true;
}

