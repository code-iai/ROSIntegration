#include "Conversion/Messages/sensor_msgs/SensorMsgsJointStateConverter.h"


USensorMsgsJointStateConverter::USensorMsgsJointStateConverter()
{
	_MessageType = "sensor_msgs/JointState";
}

bool USensorMsgsJointStateConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg>& BaseMsg) 
{

	auto msg = new ROSMessages::sensor_msgs::JointState();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_joint_state(message->full_msg_bson_, "msg", msg);
}

bool USensorMsgsJointStateConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::sensor_msgs::JointState>(BaseMsg);
	*message = bson_new();
	_bson_append_joint_state(*message, CastMsg.Get());
	return true;
}