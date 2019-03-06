#include "Conversion/Messages/sensor_msgs/SensorMsgsImuConverter.h"

#include "sensor_msgs/Imu.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"

// http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html

USensorMsgsImuConverter::USensorMsgsImuConverter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_MessageType = "sensor_msgs/Imu";
}

bool USensorMsgsImuConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto i = new ROSMessages::sensor_msgs::Imu();
	BaseMsg = TSharedPtr<FROSBaseMsg>(i);

	bool KeyFound = false;
	KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(message->full_msg_bson_, TEXT("msg.header"), &i->header);
	if (!KeyFound) return false;

	KeyFound = UGeometryMsgsQuaternionConverter::_bson_extract_child_quaternion(message->full_msg_bson_, TEXT("msg.orientation"), &i->orientation);
	if (!KeyFound) return false;
	
	i->orientation_covariance = GetDoubleTArrayFromBSON("msg.orientation_covariance", message->full_msg_bson_, KeyFound);
	if (!KeyFound || i->orientation_covariance.Num() != 9) // Size of covariance, 3x3 -> array of 9 see ROS IMU msg definition at above link
		return false;
	
	KeyFound = UGeometryMsgsVector3Converter::_bson_extract_child_vector3(message->full_msg_bson_, "msg.angular_velocity",&i->angular_velocity);
	if (!KeyFound) return false;

	i->angular_velocity_covariance = GetDoubleTArrayFromBSON("msg.angular_velocity_covariance", message->full_msg_bson_, KeyFound);
	if (!KeyFound || i->angular_velocity_covariance.Num() != 9) // Size of covariance, 3x3 -> array of 9 see ROS IMU msg definition at above link
		return false;

	KeyFound = UGeometryMsgsVector3Converter::_bson_extract_child_vector3(message->full_msg_bson_, "msg.linear_acceleration",&i->linear_acceleration);
	if (!KeyFound) return false;

	i->linear_acceleration_covariance = GetDoubleTArrayFromBSON("msg.linear_acceleration_covariance", message->full_msg_bson_, KeyFound);
	if (!KeyFound || i->linear_acceleration_covariance.Num() != 9) // Size of covariance, 3x3 -> array of 9 see ROS IMU msg definition at above link
		return false;
		
	return true;
}

bool USensorMsgsImuConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto Imu = StaticCastSharedPtr<ROSMessages::sensor_msgs::Imu>(BaseMsg);

	*message = new bson_t;
	bson_init(*message);

	UStdMsgsHeaderConverter::_bson_append_header(*message, &(Imu->header));

	UGeometryMsgsQuaternionConverter::_bson_append_child_quaternion(*message, "orientation", &(Imu->orientation));
	_bson_append_double_tarray(*message, "orientation_covariance", Imu->orientation_covariance);
	UGeometryMsgsVector3Converter::_bson_append_child_vector3(*message, "angular_velocity", &(Imu->angular_velocity));
	_bson_append_double_tarray(*message, "angular_velocity_covariance", Imu->orientation_covariance);
	UGeometryMsgsVector3Converter::_bson_append_child_vector3(*message, "linear_acceleration", &(Imu->linear_acceleration));
	_bson_append_double_tarray(*message, "linear_acceleration_covariance", Imu->linear_acceleration_covariance);

	return true;
}

