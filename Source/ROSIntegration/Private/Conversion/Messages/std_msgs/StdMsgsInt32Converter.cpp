#include "Conversion/Messages/std_msgs/StdMsgsInt32Converter.h"


UStdMsgsInt32Converter::UStdMsgsInt32Converter()
{
	_MessageType = "std_msgs/Int32";
}

bool UStdMsgsInt32Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	bool KeyFound = false;

	int32 Data = GetInt32FromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::Int32(Data));
	return true;
}

bool UStdMsgsInt32Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	auto Int32Message = StaticCastSharedPtr<ROSMessages::std_msgs::Int32>(BaseMsg);
	*message = BCON_NEW(
		"data", BCON_INT32(Int32Message->_Data)
	);
	return true;
}
