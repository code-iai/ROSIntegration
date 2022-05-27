#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/nav_msgs/NavMsgsMapMetaDataConverter.h"
#include "nav_msgs/OccupancyGrid.h"
#include "NavMsgsOccupancyGridConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UNavMsgsOccupancyGridConverter : public UBaseMessageConverter
{
	GENERATED_BODY()
	
public:
	UNavMsgsOccupancyGridConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_occupancy_grid(bson_t *b, FString key, ROSMessages::nav_msgs::OccupancyGrid *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		if (!UNavMsgsMapMetaDataConverter::_bson_extract_child_map_meta_data(b, key + ".info", &msg->info)) return false;
		msg->data = GetInt32TArrayFromBSON(key + ".data", b, KeyFound); if (!KeyFound) return false;
		
		return true;
	}

	static void _bson_append_child_occupancy_grid(bson_t *b, const char *key, const ROSMessages::nav_msgs::OccupancyGrid *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_occupancy_grid(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_occupancy_grid(bson_t *b, const ROSMessages::nav_msgs::OccupancyGrid *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		UNavMsgsMapMetaDataConverter::_bson_append_child_map_meta_data(b, "info", &msg->info);
		_bson_append_int32_tarray(b, "data", msg->data);
	}
};