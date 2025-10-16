#include "MessageConverter.h"

/**
 * Constructor, set _MessageType to the type that you want to be accessible
 */
UMessageConverter::UMessageConverter()
{
	_MessageType = TEXT("std_msgs/msg/String");
}

/**
 * how the incoming message is parsed within Unreal
 * @param message The incoming message
 * @param BaseMsg 
 * @return 
 */
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

