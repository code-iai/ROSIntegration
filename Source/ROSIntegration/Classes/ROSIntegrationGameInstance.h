#pragma once

#include <CoreMinimal.h>
#include <Engine/GameInstance.h>
#include "ROSIntegrationCore.h"

#include "ROSIntegrationGameInstance.generated.h"

UCLASS()
class ROSINTEGRATION_API UROSIntegrationGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	virtual void Init() override;
	virtual void Shutdown() override;
	void BeginDestroy() override;

public:
	UPROPERTY()
	UROSIntegrationCore* ROSIntegrationCore;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	FString ROSBridgeServerHost = "127.0.0.1";

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	int32 ROSBridgeServerPort = 9090;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	bool bConnectToROS = true;

	UPROPERTY(BlueprintReadOnly, Category = "ROS")
	bool bIsConnected = false;

	UPROPERTY(EditAnywhere, Category = "ROS")
	bool bSimulateTime = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	bool bUseFixedUpdateInterval = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	float FixedUpdateInterval = 0.01666666667;

protected:
	void CheckROSBridgeHealth();

	void OnWorldTickStart(ELevelTick TickType, float DeltaTime);

	FTimerHandle TimerHandle_CheckHealth;

	bool bReconnect = false;

private:

	UPROPERTY()
	class UTopic* ClockTopic;
};
