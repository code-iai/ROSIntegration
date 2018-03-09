// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <functional>
#include <memory>
#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "ROSBaseMsg.h"
#include "ROSIntegrationCore.h"
#include "Topic.generated.h"

UCLASS()
class ROSINTEGRATION_API UTopic: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	
    bool Subscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func);

    bool Unsubscribe();

    bool Advertise();
	
    bool Unadvertise();
	
    bool Publish(TSharedPtr<FROSBaseMsg> msg);

	void BeginDestroy() override;

	void Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType);

private:
	// PIMPL
	class Impl;
	Impl* _Implementation;
};

