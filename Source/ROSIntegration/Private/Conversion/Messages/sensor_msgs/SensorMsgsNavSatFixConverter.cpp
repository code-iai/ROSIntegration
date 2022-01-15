#include "Conversion/Messages/sensor_msgs/SensorMsgsNavSatFixConverter.h"

#include "sensor_msgs/NavSatFix.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/sensor_msgs/SensorMsgsNavSatStatusConverter.h"

USensorMsgsNavSatFixConverter::USensorMsgsNavSatFixConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/NavSatFix";
}

bool USensorMsgsNavSatFixConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto nsf = new ROSMessages::sensor_msgs::NavSatFix();
	BaseMsg = TSharedPtr<FROSBaseMsg>(nsf);

	bool KeyFound = false;
	KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(message->full_msg_bson_, "msg.header", &(nsf->header));
	if (!KeyFound) return false;

	KeyFound = USensorMsgsNavSatStatusConverter::_bson_extract_child_nav_sat_status(message->full_msg_bson_, "msg.status", &(nsf->status));
	if (!KeyFound) return false;

	nsf->latitude = GetDoubleFromBSON("msg.latitude", message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	nsf->longitude = GetDoubleFromBSON("msg.longitude", message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	nsf->altitude = GetDoubleFromBSON("msg.altitude", message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	nsf->position_covariance = GetDoubleTArrayFromBSON("msg.position_covariance", message->full_msg_bson_, KeyFound);
	if (!KeyFound || nsf->position_covariance.Num() != 9) // Covariance is a 3x3 matrix, so we should have 9 elements.
		return false;

	nsf->position_covariance_type = static_cast<ROSMessages::sensor_msgs::NavSatFix::CovarianceType>(GetInt32FromBSON("msg.position_covariance_type", message->full_msg_bson_, KeyFound));
	if (!KeyFound) return false;

	return true;
}

bool USensorMsgsNavSatFixConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto nsf = StaticCastSharedPtr<ROSMessages::sensor_msgs::NavSatFix>(BaseMsg);

	*message = bson_new();

	UStdMsgsHeaderConverter::_bson_append_child_header(*message, "header", &(nsf->header));
	USensorMsgsNavSatStatusConverter::_bson_append_child_nav_sat_status(*message, "status", &(nsf->status));
	BSON_APPEND_DOUBLE(*message, "latitude", nsf->latitude);
	BSON_APPEND_DOUBLE(*message, "longitude", nsf->longitude);
	BSON_APPEND_DOUBLE(*message, "altitude", nsf->altitude);
	_bson_append_double_tarray(*message, "position_covariance", nsf->position_covariance);
	BSON_APPEND_INT32(*message, "position_covariance_type", nsf->position_covariance_type);

	return true;
}
