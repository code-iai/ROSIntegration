#include "Conversion/Messages/std_msgs/StdMsgsFloat32Converter.h"


UStdMsgsFloat32Converter::UStdMsgsFloat32Converter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "std_msgs/Float32";
}

bool UStdMsgsFloat32Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	bool KeyFound = false;

	float Data = (float)GetDoubleFromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::Float32(Data));
	return true;
}

bool UStdMsgsFloat32Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	auto Float32Message = StaticCastSharedPtr<ROSMessages::std_msgs::Float32>(BaseMsg);
	*message = BCON_NEW(
		"data", BCON_DOUBLE(Float32Message->_Data)
	);
	return true;
}
