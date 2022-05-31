#include "Conversion/Messages/sensor_msgs/SensorMsgsCameraInfoConverter.h"


USensorMsgsCameraInfoConverter::USensorMsgsCameraInfoConverter()
{
	_MessageType = "sensor_msgs/CameraInfo";
}

bool USensorMsgsCameraInfoConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) 
{
	auto msg = new ROSMessages::sensor_msgs::CameraInfo;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_camera_info(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsCameraInfoConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{

	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::CameraInfo>(BaseMsg);

	// assert(CastMsg->K.Num() == 9); // TODO: use Unreal assertions
	// assert(CastMsg->R.Num() == 9);
	// assert(CastMsg->P.Num() == 12);

	*message = bson_new();
	_bson_append_camera_info(*message, CastMsg.Get());
	return true;
}