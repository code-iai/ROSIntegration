#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "ROSIntegrationCore.h"
#include "MyPluginGameInstance.generated.h"

UCLASS()
class ROSINTEGRATION_API UMyPluginGameInstance : public UGameInstance
{
	GENERATED_BODY()

	virtual void Init() override;
	virtual void Shutdown() override;
	void BeginDestroy() override;

public:
	UPROPERTY()
	UROSIntegrationCore* _Ric;
};
