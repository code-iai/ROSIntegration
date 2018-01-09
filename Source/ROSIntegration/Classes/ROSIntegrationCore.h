#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"



#include "ROSIntegrationCore.generated.h"


UCLASS()
class ROSINTEGRATION_API UROSIntegrationCore: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	void Init(FString ROSBridgeHost, int32 ROSBridgePort);

	// You must call Init() before using this method to set upthe Implmentation correctly
	void SetWorld(UWorld* World);

	void InitSpawnManager();

	void BeginDestroy() override;


private:

	UPROPERTY()
	bool test;

	// PIMPL
	class Impl;
	Impl* _Implementation;

	friend class UTopic;
	friend class UService;


	
	bool bson_test_mode = true;
	
};

