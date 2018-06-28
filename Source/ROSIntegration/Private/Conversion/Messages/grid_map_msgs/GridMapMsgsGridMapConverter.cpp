#include "GridMapMsgsGridMapConverter.h"

UGridMapMsgsGridMapConverter::UGridMapMsgsGridMapConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "grid_map_msgs/GridMap";
}

bool UGridMapMsgsGridMapConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::grid_map_msgs::GridMap;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_grid_map(message->full_msg_bson_, "msg", p);
}

bool UGridMapMsgsGridMapConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto map = StaticCastSharedPtr<ROSMessages::grid_map_msgs::GridMap>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_grid_map(*message, map.Get());

	return true;
}
