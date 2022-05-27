#include "Conversion/Messages/sensor_msgs/SensorMsgsLaserScanConverter.h"


USensorMsgsLaserScanConverter::USensorMsgsLaserScanConverter()
{
	_MessageType = "sensor_msgs/LaserScan";
}

bool USensorMsgsLaserScanConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::LaserScan;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_laser_scan(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsLaserScanConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::LaserScan>(BaseMsg);
	*message = bson_new();
	_bson_append_laser_scan(*message, CastMsg.Get());
	return true;
}
