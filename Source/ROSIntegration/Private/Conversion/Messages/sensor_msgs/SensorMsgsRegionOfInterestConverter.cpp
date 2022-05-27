#include "Conversion/Messages/sensor_msgs/SensorMsgsRegionOfInterestConverter.h"


USensorMsgsRegionOfInterestConverter::USensorMsgsRegionOfInterestConverter()
{
	_MessageType = "sensor_msgs/RegionOfInterest";
}

bool USensorMsgsRegionOfInterestConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) 
{
	auto msg = new ROSMessages::sensor_msgs::RegionOfInterest();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_roi(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsRegionOfInterestConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::RegionOfInterest>(BaseMsg);
	*message = bson_new();
	_bson_append_roi(*message, CastMsg.Get());
	return true;
}