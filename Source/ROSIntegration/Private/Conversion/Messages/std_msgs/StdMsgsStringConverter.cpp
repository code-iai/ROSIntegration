#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"


UStdMsgsStringConverter::UStdMsgsStringConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/String";
}

bool UStdMsgsStringConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	UE_LOG(LogTemp,Warning,TEXT("std_msgs Converter"));
	return false;
}

bool UStdMsgsStringConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	UE_LOG(LogTemp, Warning, TEXT("std_msgs Converter"));

	auto StringMessage = StaticCastSharedPtr<ROSMessages::std_msgs::String>(BaseMsg);
	*message = BCON_NEW(
		"data", TCHAR_TO_UTF8(*StringMessage->_Data)
	);
	return true;
}