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

/**
* @ingroup ROS Message Types
* Which Message type to work with.
*/
UENUM(BlueprintType, Category = "ROS")
enum class EMessageType : uint8
{
    String = 0,
    Float32 = 1,
};

UCLASS(Blueprintable)
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

	void Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType, int32 QueueSize = 10);

    virtual void PostInitProperties() override;

protected:

    UFUNCTION(BlueprintImplementableEvent, Category = ROS)
        void OnConstruct();

    UFUNCTION(BlueprintImplementableEvent, Category = ROS)
        void OnStringMessage(const FString& Data);

    UFUNCTION(BlueprintImplementableEvent, Category = ROS)
        void OnFloat32Message(const float& Data);

private:

    /*
    * Subscribe to the given topic
    */
    UFUNCTION(BlueprintCallable, Category = "ROS|Topic")
    void Subscribe(const FString& TopicName, EMessageType MessageType, int32 QueueSize = 1);

	// PIMPL
	class Impl;
	Impl* _Implementation;
    
    TSharedPtr<UTopic, ESPMode::ThreadSafe> _SelfPtr;
};

