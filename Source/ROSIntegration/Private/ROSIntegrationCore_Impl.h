#pragma once

#include "ROSIntegrationCore.h"
#include "ROSIntegrationGameInstance.h"
#include "rosbridge2cpp/TCPConnection.h"
#include "rosbridge2cpp/WebsocketConnection.h"
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"

#include "SpawnManager.h"
#include "SpawnObjectMessage.h"


#include <sstream>

#define UNREAL_ROS_CHECK_KEY_FOUND \
	if (!key_found) {\
		UE_LOG(LogROS, Warning, TEXT("%s is not present in data"), *FString(UTF8_TO_TCHAR(LookupKey.c_str())));\
		return;\
	}


// PIMPL
class UImpl::Impl
{
	// hidden implementation details
	TCPConnection* _TCPConnection = nullptr;
	WebsocketConnection* _WebsocketConnection = nullptr;
	rosbridge2cpp::ROSBridge* _Ros = nullptr;
public:
	bool _bson_test_mode;

	rosbridge2cpp::ROSBridge& GetBridge() { return *_Ros; }

	UWorld* _World = nullptr;

	//UPROPERTY() // this UPROPERTY is completely useless and its ignored by the metacompiler which works only on headers  
	USpawnManager* _SpawnManager = nullptr;


	std::unique_ptr<rosbridge2cpp::ROSTopic> _SpawnArrayMessageListener;

private:
	FString _ROSBridgeHost;
	int32 _ROSBridgePort;

public: 

	void SpawnArrayMessageCallback(const ROSBridgePublishMsg& message)
	{
		if (!rosbridge2cpp::Helper::bson_has_key(*message.full_msg_bson_, "msg.markers")) {
			UE_LOG(LogROS, Warning, TEXT("msg.markers field missing from SpawnArray Message"));
			return;
		}

		bson_iter_t iter;
		bson_iter_t val;

		if (bson_iter_init(&iter, message.full_msg_bson_) && bson_iter_find_descendant(&iter, "msg.markers", &val) &&
			BSON_ITER_HOLDS_ARRAY(&val)) {
			UE_LOG(LogROS, Verbose, TEXT("Marker is included and is an array!"));
		} else {
			UE_LOG(LogROS, Verbose, TEXT("Marker is not included or isn't an array!"));
		}

		const char* key;
		bson_iter_t child;
		uint32_t array_len = 0;

		bson_iter_recurse(&val, &child);
		while (bson_iter_next(&child)) {
			key = bson_iter_key(&child);
			if (BSON_ITER_HOLDS_DOCUMENT(&child)) {
				array_len++;
			}
		}

		// Construct dot notation address to fetch data
		for (uint32_t i = 0; i < array_len; ++i) {
			double value;
			int32 ivalue;
			bool key_found;
			SpawnObjectMessage Message;

			const std::string marker_key_prefix("msg.markers.");
			std::string LookupKey;
			std::stringstream LookupKeyStream;

			// TODO make this more generic or use other BSON library
			// Use Templates instead of different functions?
			// Make a Map that contains a tuple of (key, typ,e reference_to_variable_in_message) to automate everything?
			LookupKeyStream << marker_key_prefix << i << ".pose.position.x";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Pose._Position.X = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".pose.position.y";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Pose._Position.Y = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".pose.position.z";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Pose._Position.Z = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".pose.orientation.x";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Pose._Orientation.X = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".pose.orientation.y";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Pose._Orientation.Y = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".pose.orientation.z";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Pose._Orientation.Z = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".pose.orientation.w";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Pose._Orientation.W = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".type";
			LookupKey = LookupKeyStream.str();
			ivalue = rosbridge2cpp::Helper::get_int32_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Type = ivalue;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".id";
			LookupKey = LookupKeyStream.str();
			ivalue = rosbridge2cpp::Helper::get_int32_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Id = ivalue;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".action";
			LookupKey = LookupKeyStream.str();
			ivalue = rosbridge2cpp::Helper::get_int32_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Action = ivalue;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".scale.x";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Scale.X = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".scale.y";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Scale.Y = value;

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".scale.z";
			LookupKey = LookupKeyStream.str();
			value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Scale.Z = value;

			// Color
			double R, G, B, CAlpha;
			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".color.r";
			LookupKey = LookupKeyStream.str();
			R = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".color.g";
			LookupKey = LookupKeyStream.str();
			G = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".color.b";
			LookupKey = LookupKeyStream.str();
			B = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".color.a";
			LookupKey = LookupKeyStream.str();
			CAlpha = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND

			Message._Color = FLinearColor(R, G, B, CAlpha);

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".text";
			LookupKey = LookupKeyStream.str();
			std::string MsgText =
			rosbridge2cpp::Helper::get_utf8_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._Text = FString(MsgText.c_str());

			LookupKeyStream.str("");
			LookupKeyStream.clear();
			LookupKeyStream << marker_key_prefix << i << ".mesh_resource";
			LookupKey = LookupKeyStream.str();
			std::string MeshResource =
			rosbridge2cpp::Helper::get_utf8_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
			UNREAL_ROS_CHECK_KEY_FOUND
			Message._MeshResource = FString(MeshResource.c_str());

			UE_LOG(LogROS, Verbose, TEXT("Enqueue Message"));
			_SpawnManager->_SpawnObjectMessageQueue.Enqueue(Message);
			UE_LOG(LogROS, Verbose, TEXT("Enqueue Message Done"));
		}
	}

	std::unique_ptr<rosbridge2cpp::ROSTopic> _SpawnMessageListener;

	void SpawnMessageCallback(const ROSBridgePublishMsg& message)
	{
		UE_LOG(LogROS, Warning, TEXT("RECEIVED SPAWN MESSAGE --- Not implemented yet. Use the SpawnArray topic instead"));
	}

	Impl()
	{

	}

	~Impl()
	{
		UE_LOG(LogROS, Display, TEXT("UROSIntegrationCore ~Impl() "));
		//_World = nullptr;
		_SpawnManager = nullptr;
		if(_Ros) delete _Ros;
		if(_WebsocketConnection) delete _WebsocketConnection;
		if (_TCPConnection) delete _TCPConnection;
	}

	bool IsHealthy() const
	{
		return ((_TCPConnection != nullptr && _TCPConnection->IsHealthy()) || (_WebsocketConnection != nullptr && _WebsocketConnection->IsHealthy())) && _Ros != nullptr && _Ros->IsHealthy();
	}

	void SetWorld(UWorld* World)
	{
		_World = World;
	}

	void SetImplSpawnManager(USpawnManager* SpawnManager)
	{
		_SpawnManager = SpawnManager;
	}

	bool Init(FString protocol, FString ROSBridgeHost, int32 ROSBridgePort, bool bson_test_mode)
	{
		_bson_test_mode = bson_test_mode;

		if (protocol == "ws") {
			_WebsocketConnection = new WebsocketConnection();
			_Ros = new rosbridge2cpp::ROSBridge(*_WebsocketConnection);
		} else if (protocol == "tcp") {
			_TCPConnection = new TCPConnection();
			_Ros = new rosbridge2cpp::ROSBridge(*_TCPConnection);
		} else {
			UE_LOG(LogROS, Error, TEXT("Protocol not supported"));
		}

		if (bson_test_mode) {
			_Ros->enable_bson_mode();
		}
		_ROSBridgeHost = ROSBridgeHost;
		_ROSBridgePort = ROSBridgePort;

		bool ConnectionSuccessful = _Ros->Init(TCHAR_TO_UTF8(*ROSBridgeHost), ROSBridgePort);
		if (!ConnectionSuccessful) {
			return false;
		}

		UE_LOG(LogROS, Log, TEXT("rosbridge2cpp init successful"));

		return true;
	}

	FString GetROSBridgeHost() const
	{
		return _ROSBridgeHost;
	}

	int32 GetROSBridgePort() const
	{
		return _ROSBridgePort;
	}

	void InitSpawnManager()
	{
		// Listen to the object spawning thread
		_SpawnMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
			new rosbridge2cpp::ROSTopic(GetBridge(), "/unreal_ros/spawn_objects", "visualization_msgs/Marker"));
		_SpawnMessageListener->Subscribe(std::bind(&UImpl::Impl::SpawnMessageCallback, this, std::placeholders::_1));

		_SpawnArrayMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
			new rosbridge2cpp::ROSTopic(GetBridge(), "/unreal_ros/spawn_objects_array", "visualization_msgs/MarkerArray"));
		_SpawnArrayMessageListener->Subscribe(
			std::bind(&UImpl::Impl::SpawnArrayMessageCallback, this, std::placeholders::_1));

		//_SpawnManager = NewObject<USpawnManager>();
		_SpawnManager->_World = _World;
		_SpawnManager->_TickingActive = true;
	}
};
