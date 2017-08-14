// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "rosbridge2cpp/messages/rosbridge_publish_msg.h"
#include "bson.h"
#include "BaseMessageConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UBaseMessageConverter: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY()
	FString _MessageType;

	//For ConvertMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

};

