#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"
#include "sensor_msgs/Imu.h"
#include "SensorMsgsImuConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsImuConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsImuConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
	
	static bool _bson_extract_child_imu(bson_t *b, FString key, ROSMessages::sensor_msgs::Imu *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		if (!UGeometryMsgsQuaternionConverter::_bson_extract_child_quaternion(b, key + ".orientation", &msg->orientation)) return false;
		
		msg->orientation_covariance = GetDoubleTArrayFromBSON(key + ".orientation_covariance", b, KeyFound);
		if (!KeyFound || msg->orientation_covariance.Num() != 9) // Size of covariance, 3x3 -> array of 9 see ROS IMU msg definition at above link
			return false;
		
		if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".angular_velocity", &msg->angular_velocity)) return false;

		msg->angular_velocity_covariance = GetDoubleTArrayFromBSON(key + ".angular_velocity_covariance", b, KeyFound);
		if (!KeyFound || msg->angular_velocity_covariance.Num() != 9) // Size of covariance, 3x3 -> array of 9 see ROS IMU msg definition at above link
			return false;

		if (!UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".linear_acceleration", &msg->linear_acceleration)) return false;

		msg->linear_acceleration_covariance = GetDoubleTArrayFromBSON(key + ".linear_acceleration_covariance", b, KeyFound);
		if (!KeyFound || msg->linear_acceleration_covariance.Num() != 9) // Size of covariance, 3x3 -> array of 9 see ROS IMU msg definition at above link
			return false;

		return true;
	}

	static void _bson_append_child_imu(bson_t *b, const char *key, const ROSMessages::sensor_msgs::Imu *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_imu(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_imu(bson_t *b, const ROSMessages::sensor_msgs::Imu *msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		UGeometryMsgsQuaternionConverter::_bson_append_child_quaternion(b, "orientation", &msg->orientation);
		_bson_append_double_tarray(b, "orientation_covariance", msg->orientation_covariance);
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "angular_velocity", &msg->angular_velocity);
		_bson_append_double_tarray(b, "angular_velocity_covariance", msg->orientation_covariance);
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "linear_acceleration", &msg->linear_acceleration);
		_bson_append_double_tarray(b, "linear_acceleration_covariance", msg->linear_acceleration_covariance);
	}
};