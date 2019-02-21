#include "NavMsgsOccupancyGridConverter.h"

#include "nav_msgs/OccupancyGrid.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "NavMsgsMapMetaDataConverter.h"

UNavMsgsOccupancyGridConverter::UNavMsgsOccupancyGridConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "nav_msgs/OccupancyGrid";
}

bool UNavMsgsOccupancyGridConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto o = new ROSMessages::nav_msgs::OccupancyGrid();
	BaseMsg = TSharedPtr<FROSBaseMsg>(o);
	
	bool KeyFound = false;
	KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(message->full_msg_bson_, TEXT("msg.header"), &o->header);
	if (!KeyFound) return false;
	
	KeyFound = UNavMsgsMapMetaDataConverter::_bson_extract_child_map_meta_data(message->full_msg_bson_, TEXT("msg.info"), &o->info);
	if (!KeyFound) return false;
	
	o->data = GetInt32TArrayFromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;
	
	return true;
}

bool UNavMsgsOccupancyGridConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto grid = StaticCastSharedPtr<ROSMessages::nav_msgs::OccupancyGrid>(BaseMsg);
	
	*message = new bson_t;
	bson_init(*message);
	
	UStdMsgsHeaderConverter::_bson_append_header(*message, &(grid->header));
	
	UNavMsgsMapMetaDataConverter::_bson_append_child_map_meta_data(*message, "info", &(grid->info));
	
	_bson_append_int32_tarray(*message, "data", grid->data);
	
	return true;
}


