#include "Conversion/Messages/sensor_msgs/SensorMsgsLaserScanConverter.h"

USensorMsgsLaserScanConverter::USensorMsgsLaserScanConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/LaserScan";
}

bool USensorMsgsLaserScanConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::sensor_msgs::LaserScan;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_laser_scan(message->full_msg_bson_, "msg", p);
}

bool USensorMsgsLaserScanConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto mad = StaticCastSharedPtr<ROSMessages::sensor_msgs::LaserScan>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_laser_scan(*message, mad.Get());

	return true;
}

