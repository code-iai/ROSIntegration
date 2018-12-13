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
	auto h = StaticCastSharedPtr<ROSMessages::std_msgs::Header>(BaseMsg);

	*message = BCON_NEW(
		"seq", BCON_INT32(h->seq),
		"stamp", "{",
		"secs", BCON_INT32(h->time._Sec),
		"nsecs", BCON_INT32(h->time._NSec),
		"}",
		"frame_id", BCON_UTF8(TCHAR_TO_ANSI(*h->frame_id))
	);

	return true;
}
