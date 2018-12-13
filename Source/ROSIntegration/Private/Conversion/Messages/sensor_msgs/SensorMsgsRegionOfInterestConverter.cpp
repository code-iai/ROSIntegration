#include "Conversion/Messages/sensor_msgs/SensorMsgsRegionOfInterestConverter.h"

#include "sensor_msgs/RegionOfInterest.h"


USensorMsgsRegionOfInterestConverter::USensorMsgsRegionOfInterestConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/RegionOfInterest";
}

bool USensorMsgsRegionOfInterestConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	UE_LOG(LogROS, Warning, TEXT("ROSIntegration: RegionOfInterest receiving not implemented yet"));
	return false;
}

bool USensorMsgsRegionOfInterestConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	UE_LOG(LogROS, Warning, TEXT("ROSIntegration: RegionOfInterest sending not implemented yet"));
	return false;
}
