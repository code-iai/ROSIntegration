#include "NavMsgsMapMetaDataConverter.h"

#include "nav_msgs/MapMetaData.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"


UNavMsgsMapMetaDataConverter::UNavMsgsMapMetaDataConverter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "nav_msgs/MapMetaData";
}

bool UNavMsgsMapMetaDataConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto p = new ROSMessages::nav_msgs::MapMetaData;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_map_meta_data(message->full_msg_bson_, "msg", p);
}

bool UNavMsgsMapMetaDataConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto map = StaticCastSharedPtr<ROSMessages::nav_msgs::MapMetaData>(BaseMsg);
	
	*message = new bson_t;
	bson_init(*message);
	_bson_append_map_meta_data(*message, map.Get());
	
	return true;
}
