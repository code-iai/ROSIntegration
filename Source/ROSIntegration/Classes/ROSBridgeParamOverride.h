#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSBridgeParamOverride.generated.h"

/**
 * If you place this actor in the world, then you may override the parameters set in the ROSIntegrationGameInstance with the ones defined here.
 * The purpose of this actor is to allow you to launch multiple UE editors on the same computer (each with a different level open) and use different rosbridge connection parameters in each level.
 * This can be especially useful if you need to run independent parallel simulations on a single computer, each using a separate rosbridge node to distribute the load.
 */
UCLASS()
class ROSINTEGRATION_API AROSBridgeParamOverride : public AActor
{
	GENERATED_BODY()
	
public:	
	AROSBridgeParamOverride()
    {
        PrimaryActorTick.bCanEverTick = false; // No need to tick
    }

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	// Protocol for connecting to the rosbridge server, use "tcp" or "ws"
	FString ROSBridgeServerProtocol = "tcp";

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	// Array of IP addresses to connect to. Each element pairs with the corresponding element in ROSBridgePorts
	TArray<FString> ROSBridgeServerHosts = {"127.0.0.1"};

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	// Array of ports to connect to. Each element pairs with the corresponding element in ROSBridgeHosts
	TArray<int32> ROSBridgeServerPorts = {9090};

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS", meta=(ClampMin = '1', ClampMax = '2'))
	uint8 ROSVersion = 1;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	bool bConnectToROS = true;

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
};