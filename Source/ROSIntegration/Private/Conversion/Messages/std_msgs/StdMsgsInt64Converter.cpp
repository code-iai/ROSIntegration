#include "Conversion/Messages/std_msgs/StdMsgsInt64Converter.h"


UStdMsgsInt64Converter::UStdMsgsInt64Converter()
{
	_MessageType = "std_msgs/Int64";
}

bool UStdMsgsInt64Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	bool KeyFound = false;

	int64 Data = GetInt64FromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::Int64(Data));
	return true;
}

bool UStdMsgsInt64Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	auto Int64Message = StaticCastSharedPtr<ROSMessages::std_msgs::Int64>(BaseMsg);
	*message = BCON_NEW(
		"data", BCON_INT32(Int64Message->_Data)
	);
	return true;
}
