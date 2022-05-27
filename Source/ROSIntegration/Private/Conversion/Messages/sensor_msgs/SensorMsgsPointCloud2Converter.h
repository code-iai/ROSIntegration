#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "sensor_msgs/PointCloud2.h"
#include "SensorMsgsPointCloud2Converter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsPointCloud2Converter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsPointCloud2Converter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_point_cloud2(bson_t *b, FString key, ROSMessages::sensor_msgs::PointCloud2 *msg)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->height = GetInt32FromBSON(key + ".height", b, KeyFound); if (!KeyFound) return false;
		msg->width = GetInt32FromBSON(key + ".width", b, KeyFound); if (!KeyFound) return false;

		msg->fields = GetTArrayFromBSON<ROSMessages::sensor_msgs::PointCloud2::PointField>(key + ".fields", b, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) {
			ROSMessages::sensor_msgs::PointCloud2::PointField ret;
			ret.name = GetFStringFromBSON(subKey + ".name", subMsg, subKeyFound);
			ret.offset = GetInt32FromBSON(subKey + ".offset", subMsg, subKeyFound);
			ret.datatype = static_cast<ROSMessages::sensor_msgs::PointCloud2::PointField::EType>( GetInt32FromBSON(subKey + ".datatype", subMsg, subKeyFound) );
			ret.count = GetInt32FromBSON(subKey + ".count", subMsg, subKeyFound);
			return ret;
		});
		if (!KeyFound) return false;

		msg->is_bigendian = GetBoolFromBSON(key + ".is_bigendian", b, KeyFound); if (!KeyFound) return false;
		msg->is_dense = GetBoolFromBSON(key + ".is_dense", b, KeyFound); if (!KeyFound) return false;
		msg->point_step = GetInt32FromBSON(key + ".point_step", b, KeyFound); if (!KeyFound) return false;
		msg->row_step = GetInt32FromBSON(key + ".row_step", b, KeyFound); if (!KeyFound) return false;
		msg->data_ptr = GetBinaryFromBSON(key + ".data", b, KeyFound); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_point_cloud2(bson_t *b, const char *key, const ROSMessages::sensor_msgs::PointCloud2 *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_point_cloud2(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_point_cloud2(bson_t *b, const ROSMessages::sensor_msgs::PointCloud2 *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_INT32(b, "height", msg->height);
		BSON_APPEND_INT32(b, "width", msg->width);

		_bson_append_tarray<ROSMessages::sensor_msgs::PointCloud2::PointField>(b, "fields", msg->fields, [] (bson_t* msg, const char* key, const ROSMessages::sensor_msgs::PointCloud2::PointField& point_field)
		{
			bson_t PointField;
			BSON_APPEND_DOCUMENT_BEGIN(msg, key, &PointField);
			{
				BSON_APPEND_UTF8(&PointField, "name", TCHAR_TO_UTF8(*point_field.name));
				BSON_APPEND_INT32(&PointField, "offset", point_field.offset);
				BSON_APPEND_INT32(&PointField, "datatype", (int)point_field.datatype);
				BSON_APPEND_INT32(&PointField, "count", point_field.count);
			}
			bson_append_document_end(msg, &PointField);
		});

		BSON_APPEND_BOOL(b, "is_bigendian", msg->is_bigendian);
		BSON_APPEND_INT32(b, "point_step", msg->point_step);
		BSON_APPEND_INT32(b, "row_step", msg->row_step);
		BSON_APPEND_BINARY(b, "data", BSON_SUBTYPE_BINARY, msg->data_ptr, msg->height * msg->row_step);
		BSON_APPEND_BOOL(b, "is_dense", msg->is_dense);
	}
};