#include "GridMapMsgsGridMapInfoConverter.h"


UGridMapMsgsGridMapInfoConverter::UGridMapMsgsGridMapInfoConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "grid_map_msgs/GridMapInfo";
}

bool UGridMapMsgsGridMapInfoConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::grid_map_msgs::GridMapInfo;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_grid_map_info(message->full_msg_bson_, "msg", p);
}

bool UGridMapMsgsGridMapInfoConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Info = StaticCastSharedPtr<ROSMessages::grid_map_msgs::GridMapInfo>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);
	_bson_append_grid_map_info(*message, Info.Get());

	return true;
}
