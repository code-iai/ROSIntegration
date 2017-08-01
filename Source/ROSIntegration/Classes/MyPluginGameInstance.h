#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "MyPluginGameInstance.generated.h"

UCLASS()
class ROSINTEGRATION_API UMyPluginGameInstance : public UGameInstance
{
	GENERATED_BODY()

	virtual void Init() override;
	virtual void Shutdown() override;
};
