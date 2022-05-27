#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"
#include "grid_map_msgs/GridMapInfo.h"
#include "GridMapMsgsGridMapInfoConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGridMapMsgsGridMapInfoConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGridMapMsgsGridMapInfoConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_grid_map_info(bson_t *b, FString key, ROSMessages::grid_map_msgs::GridMapInfo *msg)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->resolution = GetDoubleFromBSON(key + ".resolution", b, KeyFound); if (!KeyFound) return false;
		msg->length_x = GetDoubleFromBSON(key + ".length_x", b, KeyFound); if (!KeyFound) return false;
		msg->length_y = GetDoubleFromBSON(key + ".length_y", b, KeyFound); if (!KeyFound) return false;
		if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".pose", &msg->pose)) return false;

		return true;
	}

	static void _bson_append_child_grid_map_info(bson_t *b, const char *key, const ROSMessages::grid_map_msgs::GridMapInfo *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_grid_map_info(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_grid_map_info(bson_t *b, const ROSMessages::grid_map_msgs::GridMapInfo *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_DOUBLE(b, "resolution", msg->resolution);
		BSON_APPEND_DOUBLE(b, "length_x", msg->length_x);
		BSON_APPEND_DOUBLE(b, "length_y", msg->length_y);
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "pose", &msg->pose);
	}
};