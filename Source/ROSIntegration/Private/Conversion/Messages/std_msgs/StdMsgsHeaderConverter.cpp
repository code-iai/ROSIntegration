#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"

#include "std_msgs/Header.h"


UStdMsgsHeaderConverter::UStdMsgsHeaderConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/Header";
}

bool UStdMsgsHeaderConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	bool KeyFound = false;

	// TODO Check if rosbridge sends UINT64 or INT32 (there is no uint32 in bson)
	int32 Seq = GetInt32FromBSON(TEXT("msg.seq"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	int32 Sec = GetInt32FromBSON(TEXT("msg.stamp.sec"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	int32 NSec = GetInt32FromBSON(TEXT("msg.stamp.nsec"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	FString FrameID = GetFStringFromBSON(TEXT("msg.frame_id"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;
	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::Header(Seq, FROSTime(Sec,NSec), FrameID));
	return true;

}

bool UStdMsgsHeaderConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {

	auto ConcreteMessage = StaticCastSharedPtr<ROSMessages::std_msgs::Header>(BaseMsg);
	*message = BCON_NEW(
		"seq", BCON_INT32(ConcreteMessage->seq),
		"stamp", "{",
			"secs", BCON_INT32(ConcreteMessage->time._Sec),
			"nsecs", BCON_INT32(ConcreteMessage->time._NSec),
		"}",
		"frame_id", BCON_UTF8(TCHAR_TO_ANSI(*ConcreteMessage->frame_id))

	);

	return true;

}