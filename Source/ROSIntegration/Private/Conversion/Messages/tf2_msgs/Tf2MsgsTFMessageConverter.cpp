#include "Conversion/Messages/tf2_msgs/Tf2MsgsTFMessageConverter.h"

#include "tf2_msgs/TFMessage.h"


UTf2MsgsTFMessageConverter::UTf2MsgsTFMessageConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "tf2_msgs/TFMessage";
}

bool UTf2MsgsTFMessageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	UE_LOG(LogROS, Warning, TEXT("ROSIntegration: TFMessage receiving not implemented yet"));
	return false;
}

bool UTf2MsgsTFMessageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	auto TFMessage = StaticCastSharedPtr<ROSMessages::tf2_msgs::TFMessage>(BaseMsg);
	if (TFMessage->transforms.Num() == 0) {
		UE_LOG(LogROS, Warning, TEXT("No transform saved in TFMessage. Can't convert message"));
		return false;
	}

	auto FirstTFMessage = TFMessage->transforms[0];

	*message = BCON_NEW(
		"transforms",
		"[",
		"{",
		"header", "{",
		"seq", BCON_INT32(FirstTFMessage.header.seq),
		"stamp", "{",
		"secs", BCON_INT32(FirstTFMessage.header.time._Sec),
		"nsecs", BCON_INT32(FirstTFMessage.header.time._NSec),
		"}",
		"frame_id", BCON_UTF8(TCHAR_TO_ANSI(*FirstTFMessage.header.frame_id)),
		"}",
		"child_frame_id", BCON_UTF8(TCHAR_TO_ANSI(*FirstTFMessage.child_frame_id)),
		"transform", "{",
		"translation", "{",
		"x", BCON_DOUBLE(FirstTFMessage.transform.translation.x),
		"y", BCON_DOUBLE(FirstTFMessage.transform.translation.y),
		"z", BCON_DOUBLE(FirstTFMessage.transform.translation.z),
		"}",
		"rotation", "{",
		"x", BCON_DOUBLE(FirstTFMessage.transform.rotation.x),
		"y", BCON_DOUBLE(FirstTFMessage.transform.rotation.y),
		"z", BCON_DOUBLE(FirstTFMessage.transform.rotation.z),
		"w", BCON_DOUBLE(FirstTFMessage.transform.rotation.w),
		"}",
		"}",
		"}",
		"]"
	);

	return true;
}
