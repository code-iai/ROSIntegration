// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "rosbridge2cpp/messages/rosbridge_publish_msg.h"
#include "bson.h"
#include "BaseResponseConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UBaseResponseConverter: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY()
	FString _ServiceType;
};

