#include "ROSIntegrationCore.h"
#include "ROSIntegrationCore_Impl.h"
#include "ROSIntegrationGameInstance.h"
#include "rosbridge2cpp/TCPConnection.h"
#include "rosbridge2cpp/WebsocketConnection.h"
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

// Moved to ROSIntegrationCore_Impl.h

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

bool UROSIntegrationCore::Init(FString protocol, FString ROSBridgeHost, int32 ROSBridgePort) {
	UE_LOG(LogROS, Verbose, TEXT("CALLING INIT ON RIC IMPL()!"));

	// ROSVersion = ROSVersionIn;
	if(!_SpawnManager)	_SpawnManager = NewObject<USpawnManager>(USpawnManager::StaticClass()); // moved here from UImpl::Init()

	//_Implementation = new UImpl::Impl;
	if (!_Implementation)
	{
		_Implementation = NewObject<UImpl>(UImpl::StaticClass());
		_Implementation->Init();
		_Implementation->SetImplSpawnManager(_SpawnManager);
	}
	return _Implementation->Get()->Init(protocol, ROSBridgeHost, ROSBridgePort, _bson_test_mode);
}

bool UROSIntegrationCore::IsHealthy() const
{
	return _Implementation->Get()->IsHealthy();
}

FString UROSIntegrationCore::GetROSBridgeHost() const
{
	return _Implementation->Get()->GetROSBridgeHost();
}

int32 UROSIntegrationCore::GetROSBridgePort() const
{
	return _Implementation->Get()->GetROSBridgePort();
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
