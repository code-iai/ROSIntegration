#pragma once

#include <CoreMinimal.h>
#include <Engine/GameInstance.h>
#include <Engine/EngineTypes.h>
#include <Runtime/Launch/Resources/Version.h>

#include "ROSIntegrationGameInstance.generated.h"

// Lets the game instance share with any bound delegates that the ROS connection status has changed
// DECLARE_MULTICAST_DELEGATE_OneParam(FOnROSConnectionStatus, bool /*IsConnected*/); // OLD
DECLARE_MULTICAST_DELEGATE_TwoParams(FOnROSConnectionStatus, int32 /*NumConnectedServers*/,  int32 /*NumDisconnectedServers*/);

UCLASS()
class ROSINTEGRATION_API UROSIntegrationGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	virtual void Init() override;
	virtual void Shutdown() override;
	virtual void BeginDestroy() override;

	// Return a pointer to the ROS connection based on the index
	class UROSIntegrationCore* GetROSConnectionFromID(int32 ID);

public:
	UPROPERTY()
	// Orginal object for connecting to rosbridge. Keeping for legacy reasons. Will be equivalent to ROSConnection[0].
	class UROSIntegrationCore* ROSIntegrationCore = nullptr;

	UPROPERTY()
	// Array of UROSIntegrationCore pointers that create the connections to rosbridge
	TArray<class UROSIntegrationCore*> ROSConnections;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	// Protocol for connecting to the rosbridge server, use "tcp" or "ws"
	FString ROSBridgeServerProtocol = "tcp";

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	// Array of IP adresses to connect to. Each element pairs with the corresponding element in ROSBridgeServerPorts
	TArray<FString> ROSBridgeServerHosts = {"127.0.0.1"};

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	// Array of ports to connect to. Each element pairs with the corresponding element in ROSBridgeServerHosts
	TArray<int32> ROSBridgeServerPorts = {9090};

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS", meta=(ClampMin = '1', ClampMax = '2'))
	uint8 ROSVersion = 1;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	bool bConnectToROS = true;

	UPROPERTY(BlueprintReadOnly, Category = "ROS")
	TArray<bool> ConnectedToROSBridge;

	int32 NumROSBridgeServers = 0;

	UPROPERTY(EditAnywhere, Category = "ROS")
	bool bSimulateTime = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	bool bUseFixedUpdateInterval = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS", Meta = (EditCondition = "bUseFixedUpdateInterval"))
	float FixedUpdateInterval = 0.01666666667;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	bool bCheckHealth = true;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS", Meta = (EditCondition = "bCheckHealth"))
	float CheckHealthInterval = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	FString ClockTopicName = "/clock";

	FOnROSConnectionStatus OnROSConnectionStatus;

	bool bUsingOverrideParameters = false;

protected:
	// Attempt to make a ROS connection to the given server's index (ID)
	void EstablishROSConnection(int32 ID);
	
	void CheckROSBridgeHealth();

	void ShutdownAllROSObjects();

	void MarkAllROSObjectsAsDisconnected();

	// Mark ROS objects as disconnected if associated with the given rosbridge server's index (ID)
	void MarkROSObjectsAsDisconnected(int32 ID);

	// Reconnect to all ROS objects associated with the given rosbridge server's index (ID)
	void ReconnectToROSObjects(int32 ID);

#if ENGINE_MINOR_VERSION > 23 || ENGINE_MAJOR_VERSION >4
	virtual void OnWorldTickStart(UWorld * World, ELevelTick TickType, float DeltaTime);
#else 
	virtual void OnWorldTickStart(ELevelTick TickType, float DeltaTime);
#endif

	FTimerHandle TimerHandle_CheckHealth;
	bool bTimerSet = false;  // has the time been set?

	bool bReconnect = false;

	FCriticalSection initMutex_;

	UPROPERTY()
	class UTopic* ClockTopic = nullptr;

	bool bAddedOnWorldTickDelegate = false;
};


template<class T>
class FLockGuard
{
	T* lockable_;
public:
	FLockGuard(T* lockable) : lockable_(lockable) { lockable->Lock(); }
	~FLockGuard() { lockable_->Unlock(); }
};
typedef FLockGuard<FCriticalSection> FLocker;

