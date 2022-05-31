#include "Conversion/Messages/nav_msgs/NavMsgsOccupancyGridConverter.h"


UNavMsgsOccupancyGridConverter::UNavMsgsOccupancyGridConverter()
{
	_MessageType = "nav_msgs/OccupancyGrid";
}

bool UNavMsgsOccupancyGridConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::nav_msgs::OccupancyGrid();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_occupancy_grid(message->full_msg_bson_, "msg", msg);
}

bool UNavMsgsOccupancyGridConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::nav_msgs::OccupancyGrid>(BaseMsg);
	*message = bson_new();
	_bson_append_occupancy_grid(*message, CastMsg.Get());
	return true;
}