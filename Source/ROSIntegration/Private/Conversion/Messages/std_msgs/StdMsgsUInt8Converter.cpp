#include "Conversion/Messages/std_msgs/StdMsgsUInt8Converter.h"


UStdMsgsUInt8Converter::UStdMsgsUInt8Converter()
{
	_MessageType = "std_msgs/UInt8";
}

bool UStdMsgsUInt8Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	bool KeyFound = false;

	int32 DataAsInt32 = GetInt32FromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;
	uint8 Data = static_cast<uint8>(DataAsInt32);
	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::UInt8(Data));
	return true;
}

bool UStdMsgsUInt8Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	auto UInt8Message = StaticCastSharedPtr<ROSMessages::std_msgs::UInt8>(BaseMsg);
	int32 DataAsInt32 = static_cast<int32>(UInt8Message->_Data);
	*message = BCON_NEW(
		"data", BCON_INT32(DataAsInt32)
	);
	return true;
}
