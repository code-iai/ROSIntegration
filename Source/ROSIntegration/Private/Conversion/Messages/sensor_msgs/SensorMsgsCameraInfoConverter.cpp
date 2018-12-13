#include "Conversion/Messages/sensor_msgs/SensorMsgsCameraInfoConverter.h"

#include "sensor_msgs/CameraInfo.h"


USensorMsgsCameraInfoConverter::USensorMsgsCameraInfoConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/CameraInfo";
}

bool USensorMsgsCameraInfoConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	UE_LOG(LogROS, Warning, TEXT("ROSIntegration: CameraInfo receiving not implemented yet"));
	return false;
}

bool USensorMsgsCameraInfoConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {

	auto CameraInfo = StaticCastSharedPtr<ROSMessages::sensor_msgs::CameraInfo>(BaseMsg);

	assert(CameraInfo->D.Num() >= 5); // TODO: use Unreal assertions
	assert(CameraInfo->K.Num() >= 9);
	assert(CameraInfo->R.Num() >= 9);
	assert(CameraInfo->P.Num() >= 12);

	*message = BCON_NEW(
		"header", "{",
		"seq", BCON_INT32(CameraInfo->header.seq),
		"stamp", "{",
		"secs", BCON_INT32(CameraInfo->header.time._Sec),
		"nsecs", BCON_INT32(CameraInfo->header.time._NSec),
		"}",
		"frame_id", BCON_UTF8(TCHAR_TO_UTF8(*CameraInfo->header.frame_id)),
		"}",
		"height", BCON_INT32(CameraInfo->height),
		"width", BCON_INT32(CameraInfo->width),
		"distortion_model", BCON_UTF8(TCHAR_TO_UTF8(*CameraInfo->distortion_model)),
		"D", "[", BCON_DOUBLE(CameraInfo->D[0]), BCON_DOUBLE(CameraInfo->D[1]), BCON_DOUBLE(CameraInfo->D[2]), BCON_DOUBLE(CameraInfo->D[3]), BCON_DOUBLE(CameraInfo->D[4]), "]",
		"K", "[", BCON_DOUBLE(CameraInfo->K[0]), BCON_DOUBLE(CameraInfo->K[1]), BCON_DOUBLE(CameraInfo->K[2]), BCON_DOUBLE(CameraInfo->K[3]), BCON_DOUBLE(CameraInfo->K[4]), BCON_DOUBLE(CameraInfo->K[5]), BCON_DOUBLE(CameraInfo->K[6]), BCON_DOUBLE(CameraInfo->K[7]), BCON_DOUBLE(CameraInfo->K[8]), "]",
		"R", "[", BCON_DOUBLE(CameraInfo->R[0]), BCON_DOUBLE(CameraInfo->R[1]), BCON_DOUBLE(CameraInfo->R[2]), BCON_DOUBLE(CameraInfo->R[3]), BCON_DOUBLE(CameraInfo->R[4]), BCON_DOUBLE(CameraInfo->R[5]), BCON_DOUBLE(CameraInfo->R[6]), BCON_DOUBLE(CameraInfo->R[7]), BCON_DOUBLE(CameraInfo->R[8]), "]",
		"P", "[", BCON_DOUBLE(CameraInfo->P[0]), BCON_DOUBLE(CameraInfo->P[1]), BCON_DOUBLE(CameraInfo->P[2]), BCON_DOUBLE(CameraInfo->P[3]), BCON_DOUBLE(CameraInfo->P[4]), BCON_DOUBLE(CameraInfo->P[5]), BCON_DOUBLE(CameraInfo->P[6]), BCON_DOUBLE(CameraInfo->P[7]), BCON_DOUBLE(CameraInfo->P[8]), BCON_DOUBLE(CameraInfo->P[9]), BCON_DOUBLE(CameraInfo->P[10]), BCON_DOUBLE(CameraInfo->P[11]), "]",
		"binning_x", BCON_INT32(CameraInfo->binning_x),
		"binning_y", BCON_INT32(CameraInfo->binning_y),
		"roi",
		"{",
		"x_offset", BCON_INT32(CameraInfo->roi.x_offset),
		"y_offset", BCON_INT32(CameraInfo->roi.y_offset),
		"height", BCON_INT32(CameraInfo->roi.height),
		"width", BCON_INT32(CameraInfo->roi.width),
		"do_rectify", BCON_BOOL(CameraInfo->roi.do_rectify),
		"}"
	);

	return true;
}
