// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "Service.generated.h"

UCLASS()
	class ROSINTEGRATION_API UService : public UObject
{
	GENERATED_BODY()
public:
	void doAnything();

private:

	UPROPERTY()
		bool test;

};