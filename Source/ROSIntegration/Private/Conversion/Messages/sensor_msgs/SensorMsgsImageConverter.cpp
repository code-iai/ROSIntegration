#include "Conversion/Messages/sensor_msgs/SensorMsgsImageConverter.h"


USensorMsgsImageConverter::USensorMsgsImageConverter()
{
	_MessageType = "sensor_msgs/Image";
}

bool USensorMsgsImageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::Image;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_image(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsImageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::Image>(BaseMsg);
	*message = bson_new();
	_bson_append_image(*message, CastMsg.Get());
	return true;
}