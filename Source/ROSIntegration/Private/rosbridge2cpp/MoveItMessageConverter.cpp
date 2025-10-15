#include "MoveItMessageConverter.h"


UMoveItMessageConverter::UMoveItMessageConverter()
{
	// TODO: fix this to whatever the data type is
	_MessageType = "unitree_arm/msg/ArmString";
}

UMoveItMessageConverter::~UMoveItMessageConverter()
{
	
}

// TODO: Adjust to convert from whatever incoming message is
bool UMoveItMessageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg>& BaseMsg)
{
	bool KeyFound = false;

	// TODO: Adjust so that you're actually getting the data from the correct field
	FString Data = GetFStringFromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::String(Data));
	return true;
}

bool UMoveItMessageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto StringMessage = StaticCastSharedPtr<ROSMessages::std_msgs::String>(BaseMsg);
	*message = BCON_NEW(
		"data", TCHAR_TO_UTF8(*StringMessage->_Data)
	);
	return true;
}


