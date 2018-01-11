#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "ROSIntegrationCore.h"


#include "ROSIntegrationGameInstance.generated.h"

UCLASS()
class ROSINTEGRATION_API UROSIntegrationGameInstance : public UGameInstance
{
	GENERATED_BODY()

	virtual void Init() override;
	virtual void Shutdown() override;
	void BeginDestroy() override;

public:
	UPROPERTY()
	UROSIntegrationCore* _Ric;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	FString ROSBridgeServerHost = "127.0.0.1";

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	int32 ROSBridgeServerPort = 9090;
};
