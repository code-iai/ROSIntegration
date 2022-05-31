#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsFloat32MultiArrayConverter.h"
#include "Conversion/Messages/grid_map_msgs/GridMapMsgsGridMapInfoConverter.h"
#include "grid_map_msgs/GridMap.h"
#include "GridMapMsgsGridMapConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGridMapMsgsGridMapConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UGridMapMsgsGridMapConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_grid_map(bson_t *b, FString key, ROSMessages::grid_map_msgs::GridMap *msg)
	{
		bool KeyFound = false;

		if (!UGridMapMsgsGridMapInfoConverter::_bson_extract_child_grid_map_info(b, key + ".info", &msg->info)) return false;

		msg->layers = GetTArrayFromBSON<FString>(key + ".layers", b, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetFStringFromBSON(subKey, subMsg, subKeyFound, false); });
		msg->basic_layers = GetTArrayFromBSON<FString>(key + ".basic_layers", b, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetFStringFromBSON(subKey, subMsg, subKeyFound, false); });

		msg->data = GetTArrayFromBSON<ROSMessages::std_msgs::Float32MultiArray>(key + ".data", b, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) {
			ROSMessages::std_msgs::Float32MultiArray ret;
			subKeyFound = UStdMsgsFloat32MultiArrayConverter::_bson_extract_child_float_multi_array(subMsg, subKey, &ret, false);
			return ret;
		});
		if (!KeyFound) return false;

		msg->outer_start_index = GetInt32FromBSON(key + ".outer_start_index", b, KeyFound); if (!KeyFound) return false;
		msg->inner_start_index = GetInt32FromBSON(key + ".inner_start_index", b, KeyFound); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_grid_map(bson_t *b, const char *key, ROSMessages::grid_map_msgs::GridMap *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_grid_map(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_grid_map(bson_t *b, const ROSMessages::grid_map_msgs::GridMap *msg)
	{
		UGridMapMsgsGridMapInfoConverter::_bson_append_child_grid_map_info(b, "info", &msg->info);

		_bson_append_tarray<FString>(b, "layers", msg->layers, [](bson_t *subb, const char *subKey, FString str) { BSON_APPEND_UTF8(subb, subKey, TCHAR_TO_UTF8(*str)); });
		_bson_append_tarray<FString>(b, "basic_layers", msg->basic_layers, [](bson_t *subb, const char *subKey, FString str) { BSON_APPEND_UTF8(subb, subKey, TCHAR_TO_UTF8(*str)); });

		_bson_append_tarray<ROSMessages::std_msgs::Float32MultiArray>(b, "data", msg->data, [](bson_t *subB, const char *subKey, ROSMessages::std_msgs::Float32MultiArray fma) {
			UStdMsgsFloat32MultiArrayConverter::_bson_append_child_float_multi_array(subB, subKey, &fma);
		});

		BSON_APPEND_INT32(b, "outer_start_index", msg->outer_start_index);
		BSON_APPEND_INT32(b, "inner_start_index", msg->inner_start_index);
	}
};