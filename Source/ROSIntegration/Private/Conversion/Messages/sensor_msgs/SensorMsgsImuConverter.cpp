#include "Conversion/Messages/sensor_msgs/SensorMsgsImuConverter.h"


USensorMsgsImuConverter::USensorMsgsImuConverter()
{
	_MessageType = "sensor_msgs/Imu";
}

bool USensorMsgsImuConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::Imu();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_imu(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsImuConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::Imu>(BaseMsg);
	*message = bson_new();
	_bson_append_imu(*message, CastMsg.Get());
	return true;
}
