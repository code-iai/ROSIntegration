#include "Conversion/Messages/sensor_msgs/SensorMsgsImageConverter.h"


USensorMsgsImageConverter::USensorMsgsImageConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/Image";
}

bool USensorMsgsImageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::sensor_msgs::Image;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_image(message->full_msg_bson_, "msg", p);
}

bool USensorMsgsImageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Image = StaticCastSharedPtr<ROSMessages::sensor_msgs::Image>(BaseMsg);

	*message = BCON_NEW(
	"header", "{",
	"seq", BCON_INT32(Image->header.seq),
	"stamp", "{",
	"secs", BCON_INT32(Image->header.time._Sec),
	"nsecs", BCON_INT32(Image->header.time._NSec),
	"}",
	"frame_id", BCON_UTF8(TCHAR_TO_UTF8(*Image->header.frame_id)),
	"}",
	"height", BCON_INT32(Image->height),
	"width", BCON_INT32(Image->width),
	"encoding", BCON_UTF8(TCHAR_TO_UTF8(*Image->encoding)),
	"step", BCON_INT32(Image->step)
	);
	bson_append_binary(*message, "data", -1, BSON_SUBTYPE_BINARY, Image->data, Image->height * Image->step );
	return true;
}
