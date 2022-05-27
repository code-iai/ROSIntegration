#include "Conversion/Messages/sensor_msgs/SensorMsgsNavSatFixConverter.h"


USensorMsgsNavSatFixConverter::USensorMsgsNavSatFixConverter()
{
	_MessageType = "sensor_msgs/NavSatFix";
}

bool USensorMsgsNavSatFixConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::NavSatFix();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_nav_sat_fix(message->full_msg_bson_, "msg", msg);
	
}

bool USensorMsgsNavSatFixConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::NavSatFix>(BaseMsg);
	*message = bson_new();
	_bson_append_nav_sat_fix(*message, CastMsg.Get());
	return true;
}