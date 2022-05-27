#include "Conversion/Messages/BaseMessageConverter.h"


UBaseMessageConverter::UBaseMessageConverter()
{
}

bool UBaseMessageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	return false;
}

bool UBaseMessageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	return false;
}
