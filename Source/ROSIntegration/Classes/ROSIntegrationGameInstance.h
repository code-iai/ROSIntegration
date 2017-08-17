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
};
