#include "GridMapMsgsGridMapInfoConverter.h"


UGridMapMsgsGridMapInfoConverter::UGridMapMsgsGridMapInfoConverter()
{
	_MessageType = "grid_map_msgs/GridMapInfo";
}

bool UGridMapMsgsGridMapInfoConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::grid_map_msgs::GridMapInfo;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_grid_map_info(message->full_msg_bson_, "msg", msg);
}

bool UGridMapMsgsGridMapInfoConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::grid_map_msgs::GridMapInfo>(BaseMsg);
	*message = bson_new();
	_bson_append_grid_map_info(*message, CastMsg.Get());
	return true;
}