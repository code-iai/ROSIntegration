// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "ROSIntegrationCore.h"
#include "ROSBaseServiceRequest.h"
#include "ROSBaseServiceResponse.h"

#include <functional>

#include "Service.generated.h"

UCLASS()
	class ROSINTEGRATION_API UService : public UObject
{
	GENERATED_UCLASS_BODY()
public:
	void doAnything();

	void Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType);

	//void Advertise(FunVrROSCallServiceMsgrROSServiceResponseMsg callback);

	//// Unadvertise an advertised service
	//// Will do nothing if no service has been advertised before in this instance
	//void Unadvertise();

	//// TODO failedCallback parameter
	//// Call a ROS-Service
	//// The given callback variable will be called when the service reply
	//// has been received by ROSBridge. It will passed the received data to the callback.
	//// The whole content of the "request" parameter will be send as the "args"
	//// argument of the Service Request
	void CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse);


private:

	UPROPERTY()
		bool test;




	// PIMPL
	class Impl;
	Impl* _Implementation;

};