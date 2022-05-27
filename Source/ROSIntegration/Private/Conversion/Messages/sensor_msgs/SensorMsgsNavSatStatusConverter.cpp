#include "Conversion/Messages/sensor_msgs/SensorMsgsNavSatStatusConverter.h"


USensorMsgsNavSatStatusConverter::USensorMsgsNavSatStatusConverter()
{
	_MessageType = "sensor_msgs/NavSatStatus";
}

bool USensorMsgsNavSatStatusConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::NavSatStatus();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_nav_sat_status(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsNavSatStatusConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::NavSatStatus>(BaseMsg);
	*message = bson_new();
	_bson_append_nav_sat_status(*message, CastMsg.Get());
	return true;
}