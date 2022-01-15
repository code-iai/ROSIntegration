#include "Conversion/Messages/sensor_msgs/SensorMsgsNavSatStatusConverter.h"

USensorMsgsNavSatStatusConverter::USensorMsgsNavSatStatusConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/NavSatStatus";
}

bool USensorMsgsNavSatStatusConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto nss = new ROSMessages::sensor_msgs::NavSatStatus();
	BaseMsg = TSharedPtr<FROSBaseMsg>(nss);
	return _bson_extract_child_nav_sat_status(message->full_msg_bson_, "msg", nss);
}

bool USensorMsgsNavSatStatusConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto nss = StaticCastSharedPtr<ROSMessages::sensor_msgs::NavSatStatus>(BaseMsg);

	*message = bson_new();
	_bson_append_nav_sat_status(*message, nss.Get());

	return true;
}
