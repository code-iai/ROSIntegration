#include "ROSIntegrationCore.h"
#include "ROSIntegrationGameInstance.h"
#include "rosbridge2cpp/TCPConnection.h"
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"

#include "SpawnManager.h"
#include "SpawnObjectMessage.h"


#include <sstream>

DEFINE_LOG_CATEGORY(LogROS);

#define UNREAL_ROS_CHECK_KEY_FOUND \
	if (!key_found) {\
		UE_LOG(LogROS, Warning, TEXT("%s is not present in data"), *FString(UTF8_TO_TCHAR(LookupKey.c_str())));\
		return;\
	}


// PIMPL
class UImpl::Impl
{
	// hidden implementation details
public:
	bool _bson_test_mode;

	TCPConnection _Connection;
	rosbridge2cpp::ROSBridge _Ros{ _Connection };


	UWorld* _World = nullptr;

	//UPROPERTY() // this UPROPERTY is completely useless and its ignored by the metacompiler which works only on headers  
	USpawnManager* _SpawnManager = nullptr;


	std::unique_ptr<rosbridge2cpp::ROSTopic> _SpawnArrayMessageListener;

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
	}

	bool IsHealthy() const
	{
		return _Connection.IsHealthy() && _Ros.IsHealthy();
	}

	void SetWorld(UWorld* World)
	{
		_World = World;
	}

	void SetImplSpawnManager(USpawnManager* SpawnManager)
	{
		_SpawnManager = SpawnManager;
	}

	bool Init(FString ROSBridgeHost, int32 ROSBridgePort, bool bson_test_mode)
	{
		_bson_test_mode = bson_test_mode;

		if (bson_test_mode) {
			_Ros.enable_bson_mode();
		}

		bool ConnectionSuccessful = _Ros.Init(TCHAR_TO_UTF8(*ROSBridgeHost), ROSBridgePort);
		if (!ConnectionSuccessful) {
			return false;
		}

		UE_LOG(LogROS, Log, TEXT("rosbridge2cpp init successful"));

		return true;
	}


	void InitSpawnManager()
	{
		// Listen to the object spawning thread
		_SpawnMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
			new rosbridge2cpp::ROSTopic(_Ros, "/unreal_ros/spawn_objects", "visualization_msgs/Marker"));
		_SpawnMessageListener->Subscribe(std::bind(&UImpl::Impl::SpawnMessageCallback, this, std::placeholders::_1));

		_SpawnArrayMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
			new rosbridge2cpp::ROSTopic(_Ros, "/unreal_ros/spawn_objects_array", "visualization_msgs/MarkerArray"));
		_SpawnArrayMessageListener->Subscribe(
			std::bind(&UImpl::Impl::SpawnArrayMessageCallback, this, std::placeholders::_1));

		//_SpawnManager = NewObject<USpawnManager>();
		_SpawnManager->_World = _World;
		_SpawnManager->_TickingActive = true;
	}
};



// - - -  - - - 


UImpl::UImpl()
{
	//Init();
}

UImpl::UImpl(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	//Init();
}

UImpl::~UImpl()
{
	UE_LOG(LogROS, Display, TEXT("UROSIntegrationCore ~UImpl() "));
}

void UImpl::Init()
{
	UE_LOG(LogROS, Display, TEXT("UImpl::Init() "));
	impl_ = TSharedPtr<Impl>(new UImpl::Impl());

}

void UImpl::BeginDestroy()
{
	UE_LOG(LogROS, Display, TEXT("UImpl::BeginDestroy() "));

	if(impl_) impl_.Reset();

	Super::BeginDestroy();
}

void UImpl::SetImplSpawnManager(USpawnManager* SpawnManager)
{
	impl_->SetImplSpawnManager(SpawnManager);
}


// - - -  - - - 


UROSIntegrationCore::UROSIntegrationCore(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	UE_LOG(LogROS, Display, TEXT("UROSIntegrationCore spawned "));
}

UROSIntegrationCore::~UROSIntegrationCore()
{
	UE_LOG(LogROS, Display, TEXT("UROSIntegrationCore ~UROSIntegrationCore() "));
}

bool UROSIntegrationCore::Init(FString ROSBridgeHost, int32 ROSBridgePort) {
	UE_LOG(LogROS, Verbose, TEXT("CALLING INIT ON RIC IMPL()!"));

	if(!_SpawnManager)	_SpawnManager = NewObject<USpawnManager>(USpawnManager::StaticClass()); // moved here from UImpl::Init()

	//_Implementation = new UImpl::Impl;
	if (!_Implementation)
	{
		_Implementation = NewObject<UImpl>(UImpl::StaticClass());
		_Implementation->Init();
		_Implementation->SetImplSpawnManager(_SpawnManager);
	}
	return _Implementation->Get()->Init(ROSBridgeHost, ROSBridgePort, _bson_test_mode);
}


bool UROSIntegrationCore::IsHealthy() const
{
	return _Implementation->Get()->IsHealthy();
}

void UROSIntegrationCore::SetWorld(UWorld* World)
{
	assert(_Implementation);
	_Implementation->Get()->SetWorld(World);
}

void UROSIntegrationCore::InitSpawnManager()
{
	assert(_Implementation);
	_Implementation->Get()->InitSpawnManager();
}


void UROSIntegrationCore::BeginDestroy()
{
	UE_LOG(LogROS, Verbose, TEXT("ROS Integration Core - BeginDestroy() - start"));
	//Super::BeginDestroy(); // TODO: Super::BeginDestroy at the end of this function! // This was the original position!

	if (_Implementation)
	{
		UE_LOG(LogROS, Verbose, TEXT("ROS Integration Core - BeginDestroy() - destroying implementation"));
		_Implementation->ConditionalBeginDestroy();
		_Implementation = nullptr;

		//delete _Implementation;
	}
	// TODO delete spawnmanager / mark for GC by RemoveFromRoot()

	Super::BeginDestroy(); // TODO: Super::BeginDestroy at the end of this function! // NEW POSITION!

	UE_LOG(LogROS, Verbose, TEXT("ROS Integration Core - BeginDestroy() - done"));
}
