#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"
#include "nav_msgs/MapMetaData.h"
#include "NavMsgsMapMetaDataConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UNavMsgsMapMetaDataConverter : public UBaseMessageConverter
{
	GENERATED_BODY()
	
public:
	UNavMsgsMapMetaDataConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
	
	static bool _bson_extract_child_map_meta_data(bson_t *b, FString key, ROSMessages::nav_msgs::MapMetaData *msg)
	{
		bool KeyFound = false;
		// int32 Sec =  GetInt32FromBSON(key + ".map_load_time.secs",  b, KeyFound);  if (!KeyFound) return false;
		// int32 NSec = GetInt32FromBSON(key + ".map_load_time.nsecs", b, KeyFound); if (!KeyFound) return false;
		// msg->map_load_time = FROSTime(Sec, NSec);
		if (!_bson_extract_child_ros_time(b, key + ".map_load_time", &msg->map_load_time)) return false;
		msg->resolution = (float)GetDoubleFromBSON(key + ".resolution", b, KeyFound);  if (!KeyFound) return false;
		msg->width = GetInt32FromBSON(key + ".width", b, KeyFound); if (!KeyFound) return false;
		msg->height = GetInt32FromBSON(key + ".height", b, KeyFound); if (!KeyFound) return false;
		if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".origin", &msg->origin)) return false;
		
		return true;
	}
	
	static void _bson_append_child_map_meta_data(bson_t *b, const char *key, const ROSMessages::nav_msgs::MapMetaData *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_map_meta_data(&child, msg);
		bson_append_document_end(b, &child);
	}
	
	static void _bson_append_map_meta_data(bson_t *b, const ROSMessages::nav_msgs::MapMetaData *msg)
	{
		// bson_t map_load_time;
		// BSON_APPEND_DOCUMENT_BEGIN(b, "map_load_time", &map_load_time);
		// BSON_APPEND_INT32(&map_load_time, "secs",  msg->map_load_time._Sec);
		// BSON_APPEND_INT32(&map_load_time, "nsecs", msg->map_load_time._NSec);
		// bson_append_document_end(b, &map_load_time);
		_bson_append_child_ros_time(b, "map_load_time", &msg->map_load_time);
		BSON_APPEND_DOUBLE(b, "resolution", msg->resolution);
		BSON_APPEND_INT32(b, "width",  msg->width);
		BSON_APPEND_INT32(b, "height", msg->height);
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "origin", &msg->origin);
	}
	
};

