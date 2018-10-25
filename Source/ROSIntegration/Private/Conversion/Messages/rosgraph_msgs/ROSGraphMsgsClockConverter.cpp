#include "ROSGraphMsgsClockConverter.h"

#include "rosgraph_msgs/Clock.h"
#include "Conversion/Messages/BaseMessageConverter.h"

UROSGraphMsgsClockConverter::UROSGraphMsgsClockConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "rosgraph_msgs/Clock";
}

bool UROSGraphMsgsClockConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto Clock = new ROSMessages::rosgraph_msgs::Clock();
	BaseMsg = TSharedPtr<FROSBaseMsg>(Clock);

	bool KeyFound = false;

	int32 Sec = GetInt32FromBSON("msg.clock.secs", message->full_msg_bson_, KeyFound);  if (!KeyFound) return false;
	int32 NSec = GetInt32FromBSON("msg.clock.nsecs", message->full_msg_bson_, KeyFound); if (!KeyFound) return false;
	Clock->_Clock = FROSTime(Sec, NSec);

	return true;
}

bool UROSGraphMsgsClockConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Clock = StaticCastSharedPtr<ROSMessages::rosgraph_msgs::Clock>(BaseMsg);

	*message = BCON_NEW(
		"clock", "{",
		"secs", BCON_INT32(Clock->_Clock._Sec),
		"nsecs", BCON_INT32(Clock->_Clock._NSec),
		"}"
	);

	return true;
}
