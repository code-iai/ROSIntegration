#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"
#include "sensor_msgs/MagneticField.h"
#include "SensorMsgsMagneticFieldConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsMagneticFieldConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsMagneticFieldConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg>& BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_magnetic_field(bson_t* b, FString key, ROSMessages::sensor_msgs::MagneticField* msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		
		if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".magnetic_field", &msg->magnetic_field)) return false;
		msg->magnetic_field_covariance = GetDoubleTArrayFromBSON(key + ".magnetic_field_covariance", b, KeyFound);
		if (!KeyFound || msg->magnetic_field_covariance.Num() != 9) // Size of covariance, 3x3 -> array of 9 see ROS magnetic field msg definition at above link
			return false;

		return true;
	}

	static void _bson_append_child_magnetic_field(bson_t* b, const char* key, const ROSMessages::sensor_msgs::MagneticField* msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_magnetic_field(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_magnetic_field(bson_t* b, const ROSMessages::sensor_msgs::MagneticField* msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "magnetic_field", &msg->magnetic_field);
		_bson_append_double_tarray(b, "magnetic_field_covariance", msg->magnetic_field_covariance);
	}
};