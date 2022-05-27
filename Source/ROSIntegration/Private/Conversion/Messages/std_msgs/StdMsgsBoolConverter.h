// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Bool.h"
#include "StdMsgsBoolConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsBoolConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsBoolConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
	
};