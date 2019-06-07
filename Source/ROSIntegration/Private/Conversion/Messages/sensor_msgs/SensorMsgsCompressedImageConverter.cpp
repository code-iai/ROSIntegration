#include "Conversion/Messages/sensor_msgs/SensorMsgsCompressedImageConverter.h"


USensorMsgsCompressedImageConverter::USensorMsgsCompressedImageConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/CompressedImage";
}

bool USensorMsgsCompressedImageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::sensor_msgs::CompressedImage;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_image(message->full_msg_bson_, "msg", p);
}

bool USensorMsgsCompressedImageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Image = StaticCastSharedPtr<ROSMessages::sensor_msgs::CompressedImage>(BaseMsg);

	*message = BCON_NEW(
		"header", "{",
		"seq", BCON_INT32(Image->header.seq),
		"stamp", "{",
		"secs", BCON_INT32(Image->header.time._Sec),
		"nsecs", BCON_INT32(Image->header.time._NSec),
		"}",
		"frame_id", BCON_UTF8(TCHAR_TO_UTF8(*Image->header.frame_id)),
		"}",
		"format", BCON_UTF8(TCHAR_TO_UTF8(*Image->format))
	);
	bson_append_binary(*message, "data", -1, BSON_SUBTYPE_BINARY, Image->data, Image->data_size);
	return true;
}
