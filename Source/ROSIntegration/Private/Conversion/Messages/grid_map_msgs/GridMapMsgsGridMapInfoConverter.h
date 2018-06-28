#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "grid_map_msgs/GridMapInfo.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"

#include "GridMapMsgsGridMapInfoConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGridMapMsgsGridMapInfoConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_grid_map_info(bson_t *b, FString key, ROSMessages::grid_map_msgs::GridMapInfo *g)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &g->header); if (!KeyFound) return false;
		g->resolution = GetDoubleFromBSON(key + ".resolution", b, KeyFound);							if (!KeyFound) return false;
		g->length_x = GetDoubleFromBSON(key + ".length_x", b, KeyFound);								if (!KeyFound) return false;
		g->length_y = GetDoubleFromBSON(key + ".length_y", b, KeyFound);								if (!KeyFound) return false;
		KeyFound = UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".pose", &g->pose);	if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_grid_map_info(bson_t *b, const char *key, ROSMessages::grid_map_msgs::GridMapInfo *g)
	{
		bson_t info;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &info);
		_bson_append_grid_map_info(&info, g);
		bson_append_document_end(b, &info);
	}

	static void _bson_append_grid_map_info(bson_t *b, ROSMessages::grid_map_msgs::GridMapInfo *g)
	{
		UStdMsgsHeaderConverter::_bson_append_header(b, &(g->header));
		BSON_APPEND_DOUBLE(b, "resolution", g->resolution);
		BSON_APPEND_DOUBLE(b, "length_x", g->length_x);
		BSON_APPEND_DOUBLE(b, "length_y", g->length_y);
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "pose", &(g->pose));
	}
};
