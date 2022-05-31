#include "NavMsgsMapMetaDataConverter.h"


UNavMsgsMapMetaDataConverter::UNavMsgsMapMetaDataConverter()
{
	_MessageType = "nav_msgs/MapMetaData";
}

bool UNavMsgsMapMetaDataConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::nav_msgs::MapMetaData;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_map_meta_data(message->full_msg_bson_, "msg", msg);
}

bool UNavMsgsMapMetaDataConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::nav_msgs::MapMetaData>(BaseMsg);
	*message = bson_new();
	_bson_append_map_meta_data(*message, CastMsg.Get());
	return true;
}