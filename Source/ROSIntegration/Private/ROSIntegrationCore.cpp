#include "ROSIntegrationCore.h"
#include "ROSIntegrationGameInstance.h"
#include "rosbridge2cpp/TCPConnection.h"
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"

#include "SpawnManager.h"
#include "SpawnObjectMessage.h"


#include <sstream>

#define UNREAL_ROS_CHECK_KEY_FOUND                                     \
    if (!key_found) {                                                  \
		UE_LOG(LogTemp, Warning, TEXT("%s is not present in data"), *UTF8_TO_TCHAR(LookupKey.c_str()) );\
        return;                                                        \
    }


// PIMPL
class UROSIntegrationCore::Impl {
	// hidden implementation details
public:
	bool _bson_test_mode;

	TCPConnection _Connection;
	rosbridge2cpp::ROSBridge _Ros{ _Connection };
	

	UWorld* _World = nullptr;

	UPROPERTY()
	USpawnManager* _SpawnManager;


	std::unique_ptr<rosbridge2cpp::ROSTopic> _SpawnArrayMessageListener;

	void SpawnArrayMessageCallback(const ROSBridgePublishMsg& message) {
		if (!rosbridge2cpp::Helper::bson_has_key(*message.full_msg_bson_, "msg.markers")) {
			UE_LOG(LogTemp, Warning, TEXT("msg.markers field missing from SpawnArray Message"));
			return;
		}

		bson_iter_t iter;
		bson_iter_t val;

		if (bson_iter_init(&iter, message.full_msg_bson_) && bson_iter_find_descendant(&iter, "msg.markers", &val) &&
			BSON_ITER_HOLDS_ARRAY(&val) ) {
			UE_LOG(LogTemp, Verbose, TEXT("Marker is included and is an array!"));
		} else {
			UE_LOG(LogTemp, Verbose, TEXT("Marker is not included or isn't an array!"));
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
		for (uint32_t i = 0; i < array_len; i++) {
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

			UE_LOG(LogTemp, Verbose, TEXT("Enqueue Message"));
			_SpawnManager->_SpawnObjectMessageQueue.Enqueue(Message);
			UE_LOG(LogTemp, Verbose, TEXT("Enqueue Message Done"));
		}

	}
	std::unique_ptr<rosbridge2cpp::ROSTopic> _SpawnMessageListener;

	void SpawnMessageCallback(const ROSBridgePublishMsg& message) {
		UE_LOG(LogTemp, Warning, TEXT("RECEIVED SPAWN MESSAGE --- Not implemented yet. Use the SpawnArray topic instead"));
	}

	Impl() {

	}

	void SetWorld(UWorld* World) {
		_World = World;
	}

	void Init(FString ROSBridgeHost, int32 ROSBridgePort, bool bson_test_mode) {
		_bson_test_mode = bson_test_mode;

		if (bson_test_mode) {
			// OUT_INFO(TEXT("BSON mode enabled"));
			_Ros.enable_bson_mode();
		}

		bool ConnectionSuccessful = _Ros.Init(TCHAR_TO_UTF8(*ROSBridgeHost), ROSBridgePort);
		if (!ConnectionSuccessful) {
			UE_LOG(LogTemp, Error, TEXT("Failed to connect to server. Please make sure that your rosbridge is running. Abort ROSBridge Init."));
			return;
		}

		UE_LOG(LogTemp, Log, TEXT("rosbridge2cpp init successful"));

		/*_Topic = new rosbridge2cpp::ROSTopic(_Ros, "/newtest", "std_msgs/String");
		_Topic->Subscribe(std::bind(&UROSIntegrationCore::Impl::MessageCallback, this, std::placeholders::_1));*/
	}


	void InitSpawnManager() {
		// Listen to the object spawning thread
		_SpawnMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
			new rosbridge2cpp::ROSTopic(_Ros, "/unreal_ros/spawn_objects", "visualization_msgs/Marker"));
		_SpawnMessageListener->Subscribe(std::bind(&UROSIntegrationCore::Impl::SpawnMessageCallback, this, std::placeholders::_1));

		_SpawnArrayMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
			new rosbridge2cpp::ROSTopic(_Ros, "/unreal_ros/spawn_objects_array", "visualization_msgs/MarkerArray"));
		_SpawnArrayMessageListener->Subscribe(
			std::bind(&UROSIntegrationCore::Impl::SpawnArrayMessageCallback, this, std::placeholders::_1));

		_SpawnManager = NewObject<USpawnManager>();
		_SpawnManager->_World = _World;
		_SpawnManager->_TickingActive = true;
	}

};


UROSIntegrationCore::UROSIntegrationCore(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

void UROSIntegrationCore::Init(FString ROSBridgeHost, int32 ROSBridgePort) {
	UE_LOG(LogTemp, Verbose, TEXT("CALLING INIT ON RIC IMPL()!"));
	_Implementation = new UROSIntegrationCore::Impl;
	_Implementation->Init(ROSBridgeHost, ROSBridgePort, bson_test_mode);
}

void UROSIntegrationCore::SetWorld(UWorld* World) {
	assert(_Implementation);
	_Implementation->SetWorld(World);
}

void UROSIntegrationCore::InitSpawnManager() {
	assert(_Implementation);
	_Implementation->InitSpawnManager();
}


void UROSIntegrationCore::BeginDestroy() {
	UE_LOG(LogTemp, Verbose, TEXT("Begin Destroy on UROSIntegrationCore called"));
	Super::BeginDestroy();

	if (_Implementation) delete _Implementation;
	// TODO delete spawnmanager / mark for GC by RemoveFromRoot()
}
