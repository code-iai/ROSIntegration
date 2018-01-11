#include "Conversion/Messages/sensor_msgs/SensorMsgsImageConverter.h"

#include "sensor_msgs/Image.h"


USensorMsgsImageConverter::USensorMsgsImageConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/Image";
}

bool USensorMsgsImageConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	UE_LOG(LogTemp, Warning, TEXT("ROSIntegration: Image receiving not implemented yet"));
	return false;
}

bool USensorMsgsImageConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {

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
	bson_append_binary(*message, "data", -1, BSON_SUBTYPE_BINARY, Image->data_ptr, Image->height * Image->step );
	return true;
}