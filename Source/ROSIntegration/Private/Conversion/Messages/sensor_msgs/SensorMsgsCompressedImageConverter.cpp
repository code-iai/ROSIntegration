#include "Conversion/Messages/sensor_msgs/SensorMsgsCompressedImageConverter.h"


USensorMsgsCompressedImageConverter::USensorMsgsCompressedImageConverter()
{
	_MessageType = "sensor_msgs/CompressedImage";
}

bool USensorMsgsCompressedImageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::CompressedImage;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_image(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsCompressedImageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::CompressedImage>(BaseMsg);
	*message = bson_new();
	_bson_append_image(*message, CastMsg.Get());
	return true;
}