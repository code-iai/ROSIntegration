#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "nav_msgs/MapMetaData.h"
#include "NavMsgsMapMetaDataConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UNavMsgsMapMetaDataConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()
	
public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
	
	// Helper function to extract a child-std_msgs/Header from a bson_t
	static bool _bson_extract_child_map_meta_data(bson_t *b, FString key, ROSMessages::nav_msgs::MapMetaData *mmd)
	{
		bool KeyFound = false;
		int32 Sec =  GetInt32FromBSON(key + ".map_load_time.secs",  b, KeyFound);  if (!KeyFound) return false;
		int32 NSec = GetInt32FromBSON(key + ".map_load_time.nsecs", b, KeyFound); if (!KeyFound) return false;
		mmd->map_load_time = FROSTime(Sec, NSec);
		
		mmd->resolution = (float)GetDoubleFromBSON(key + ".resolution", b, KeyFound);  if (!KeyFound) return false;
		
		mmd->width = GetInt32FromBSON(key + ".width", b, KeyFound); if (!KeyFound) return false;
		mmd->height = GetInt32FromBSON(key + ".height", b, KeyFound); if (!KeyFound) return false;
		
		if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".origin", &mmd->origin)) return false;
		
		return true;
	}
	
	static void _bson_append_child_map_meta_data(bson_t *b, const char *key, ROSMessages::nav_msgs::MapMetaData *mmd)
	{
		bson_t mapmetadata;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &mapmetadata);
		_bson_append_map_meta_data(&mapmetadata, mmd);
		bson_append_document_end(b, &mapmetadata);
	}
	
	static void _bson_append_map_meta_data(bson_t *b, ROSMessages::nav_msgs::MapMetaData *mmd)
	{
		bson_t map_load_time;
		BSON_APPEND_DOCUMENT_BEGIN(b, "map_load_time", &map_load_time);
		BSON_APPEND_INT32(&map_load_time, "secs",  mmd->map_load_time._Sec);
		BSON_APPEND_INT32(&map_load_time, "nsecs", mmd->map_load_time._NSec);
		bson_append_document_end(b, &map_load_time);
		
		BSON_APPEND_DOUBLE(b, "resolution", mmd->resolution);
		
		BSON_APPEND_INT32(b, "width",  mmd->width);
		BSON_APPEND_INT32(b, "height", mmd->height);
		
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "origin", &(mmd->origin));
	}
	
};

