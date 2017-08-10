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
	void doSomething();

	void Subscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func);

	void Unsubscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func);

	void Advertise();
	
	void Unadvertise();
	
	void Publish(TSharedPtr<FROSBaseMsg> msg);

	void BeginDestroy() override;

	void Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType);

private:
	// PIMPL
	class Impl;
	Impl* _Implementation;

	UPROPERTY()
	bool test;

};

