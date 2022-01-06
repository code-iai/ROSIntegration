#include "Conversion/Messages/sensor_msgs/SensorMsgsPointCloud2Converter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"

#include "sensor_msgs/PointCloud2.h"


USensorMsgsPointCloud2Converter::USensorMsgsPointCloud2Converter(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/PointCloud2";
}

bool USensorMsgsPointCloud2Converter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	bool KeyFound = false;
	bson_t *b = message->full_msg_bson_;

	auto p = new ROSMessages::sensor_msgs::PointCloud2;
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);

	KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, TEXT("msg.header"), &p->header); if (!KeyFound) return false;

	p->height = GetInt32FromBSON("msg.height", b, KeyFound); if (!KeyFound) return false;
	p->width = GetInt32FromBSON("msg.width", b, KeyFound); if (!KeyFound) return false;

	p->fields = GetTArrayFromBSON<ROSMessages::sensor_msgs::PointCloud2::PointField>(FString("msg.fields"), b, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) {
		ROSMessages::sensor_msgs::PointCloud2::PointField ret;
		ret.name = GetFStringFromBSON(subKey + ".name", subMsg, subKeyFound);
		ret.offset = GetInt32FromBSON(subKey + ".offset", subMsg, subKeyFound);
		ret.datatype = static_cast<ROSMessages::sensor_msgs::PointCloud2::PointField::EType>( GetInt32FromBSON(subKey + ".datatype", subMsg, subKeyFound) );
		ret.count = GetInt32FromBSON(subKey + ".count", subMsg, subKeyFound);
		return ret;
	});
	if (!KeyFound) return false;

	p->is_bigendian = GetBoolFromBSON("msg.is_bigendian", b, KeyFound); if (!KeyFound) return false;
	p->is_dense = GetBoolFromBSON("msg.is_dense", b, KeyFound); if (!KeyFound) return false;
	p->point_step = GetInt32FromBSON("msg.point_step", b, KeyFound); if (!KeyFound) return false;
	p->row_step = GetInt32FromBSON("msg.row_step", b, KeyFound); if (!KeyFound) return false;

	uint32_t binSize = 0;
	p->data_ptr = rosbridge2cpp::Helper::get_binary_by_key("msg.data", *b, binSize, KeyFound);

	return KeyFound;
}

bool USensorMsgsPointCloud2Converter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto PointCloud2 = StaticCastSharedPtr<ROSMessages::sensor_msgs::PointCloud2>(BaseMsg);

	*message = bson_new();
	UStdMsgsHeaderConverter::_bson_append_header(*message, &(PointCloud2->header));

	BSON_APPEND_INT32(*message, "height", PointCloud2->height);
	BSON_APPEND_INT32(*message, "width", PointCloud2->width);

	_bson_append_tarray<ROSMessages::sensor_msgs::PointCloud2::PointField>(*message, "fields", PointCloud2->fields, [] (bson_t* msg, const char* key, const ROSMessages::sensor_msgs::PointCloud2::PointField& point_field)
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

	BSON_APPEND_BOOL(*message, "is_bigendian", PointCloud2->is_bigendian);
	BSON_APPEND_INT32(*message, "point_step", PointCloud2->point_step);
	BSON_APPEND_INT32(*message, "row_step", PointCloud2->row_step);

	bson_append_binary(*message, "data", -1, BSON_SUBTYPE_BINARY, PointCloud2->data_ptr, PointCloud2->height * PointCloud2->row_step);
	BSON_APPEND_BOOL(*message, "is_dense", PointCloud2->is_dense);
	return true;
}
