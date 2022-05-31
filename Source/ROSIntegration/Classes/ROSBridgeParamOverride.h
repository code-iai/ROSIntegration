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

    // IP address of the rosbridge websocket server
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	FString ROSBridgeServerHost = "127.0.0.1";

	// Port number to access the rosbridge websocket server
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	int32 ROSBridgeServerPort = 9090;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	bool bConnectToROS = true;

	UPROPERTY(EditAnywhere, Category = "ROS")
	bool bSimulateTime = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	bool bUseFixedUpdateInterval = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	float FixedUpdateInterval = 0.01666666667;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	bool bCheckHealth = true;
};