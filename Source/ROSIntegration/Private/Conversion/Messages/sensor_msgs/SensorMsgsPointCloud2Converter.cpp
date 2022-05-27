#include "Conversion/Messages/sensor_msgs/SensorMsgsPointCloud2Converter.h"


USensorMsgsPointCloud2Converter::USensorMsgsPointCloud2Converter()
{
	_MessageType = "sensor_msgs/PointCloud2";
}

bool USensorMsgsPointCloud2Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::sensor_msgs::PointCloud2;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_point_cloud2(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsPointCloud2Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::PointCloud2>(BaseMsg);
	*message = bson_new();
	_bson_append_point_cloud2(*message, CastMsg.Get());
	return true;
}