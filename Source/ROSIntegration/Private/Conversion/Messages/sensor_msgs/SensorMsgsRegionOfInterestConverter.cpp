#include "Conversion/Messages/sensor_msgs/SensorMsgsRegionOfInterestConverter.h"


USensorMsgsRegionOfInterestConverter::USensorMsgsRegionOfInterestConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/RegionOfInterest";
}

bool USensorMsgsRegionOfInterestConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) 
{
	auto roi = new ROSMessages::sensor_msgs::RegionOfInterest();
	BaseMsg = TSharedPtr<FROSBaseMsg>(roi);
	return _bson_extract_child_roi(message->full_msg_bson_, "msg", roi);
}

bool USensorMsgsRegionOfInterestConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
	auto ROI = StaticCastSharedPtr<ROSMessages::sensor_msgs::RegionOfInterest>(BaseMsg);

	*message = bson_new();
	_bson_append_roi(*message, ROI.Get());

	return true;
}
