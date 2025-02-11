#include "Conversion/Messages/sensor_msgs/SensorMsgsMagneticFieldConverter.h"

USensorMsgsMagneticFieldConverter::USensorMsgsMagneticFieldConverter()
{
	_MessageType = "sensor_msgs/MagneticField";
}

bool USensorMsgsMagneticFieldConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg>& BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::MagneticField();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_magnetic_field(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsMagneticFieldConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::MagneticField>(BaseMsg);
	*message = bson_new();
	_bson_append_magnetic_field(*message, CastMsg.Get());
	return true;
}
