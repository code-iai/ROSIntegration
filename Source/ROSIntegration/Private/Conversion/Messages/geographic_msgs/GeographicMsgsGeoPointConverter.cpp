#include "Conversion/Messages/geographic_msgs/GeographicMsgsGeoPointConverter.h"

#include "geographic_msgs/GeoPoint.h"

UGeographicMsgsGeoPointConverter::UGeographicMsgsGeoPointConverter()
{
	_MessageType = "geographic_msgs/GeoPoint";
}

bool UGeographicMsgsGeoPointConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::geographic_msgs::GeoPoint();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_geo_point(message->full_msg_bson_, "msg", msg);
}

bool UGeographicMsgsGeoPointConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMSG = StaticCastSharedPtr<ROSMessages::geographic_msgs::GeoPoint>(BaseMsg);
	*message = bson_new();
	_bson_append_geo_point(*message, CastMSG.Get());

	return true;
}
