#include "GridMapMsgsGridMapConverter.h"

UGridMapMsgsGridMapConverter::UGridMapMsgsGridMapConverter()
{
	_MessageType = "grid_map_msgs/GridMap";
}

bool UGridMapMsgsGridMapConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::grid_map_msgs::GridMap;
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_grid_map(message->full_msg_bson_, "msg", msg);
}

bool UGridMapMsgsGridMapConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::grid_map_msgs::GridMap>(BaseMsg);
	*message = bson_new();
	_bson_append_grid_map(*message, CastMsg.Get());
	return true;
}