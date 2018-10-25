#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>

#include "ROSIntegrationCore.generated.h"

ROSINTEGRATION_API DECLARE_LOG_CATEGORY_EXTERN(LogROS, Display, All);

UCLASS()
class ROSINTEGRATION_API UROSIntegrationCore : public UObject
{
	GENERATED_UCLASS_BODY()

public:
	bool Init(FString ROSBridgeHost, int32 ROSBridgePort);

	bool IsHealthy() const;

	// You must call Init() before using this method to set upthe Implmentation correctly
	void SetWorld(UWorld* World);

	void InitSpawnManager();

	void BeginDestroy() override;


private:
	// PIMPL
	class Impl;
	Impl *_Implementation;

	bool _bson_test_mode = true;

	friend class UTopic;
	friend class UService;
};
