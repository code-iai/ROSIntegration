#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"


UStdMsgsStringConverter::UStdMsgsStringConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/String";
}

bool UStdMsgsStringConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	bool key_found = false;
	std::string data = rosbridge2cpp::Helper::get_utf8_by_key("msg.data", *message->full_msg_bson_, key_found);
	if (!key_found) {
		UE_LOG(LogTemp, Error, TEXT("Key msg.data not present in data"));
		return false;
	}
	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::String(UTF8_TO_TCHAR(data.c_str())));
	return true;
}

bool UStdMsgsStringConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	UE_LOG(LogTemp, Warning, TEXT("std_msgs Converter"));

	auto StringMessage = StaticCastSharedPtr<ROSMessages::std_msgs::String>(BaseMsg);
	*message = BCON_NEW(
		"data", TCHAR_TO_UTF8(*StringMessage->_Data)
	);
	return true;
}