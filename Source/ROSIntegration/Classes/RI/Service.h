#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
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
	void Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType);
	void BeginDestroy() override;

	bool Advertise(std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> ServiceHandler, bool HandleRequestsInGameThread);

	//// Unadvertise an advertised service
	//// Will do nothing if no service has been advertised before in this instance
	bool Unadvertise();

	// TODO failedCallback parameter
	/** Call a ROS-Service
	 * The given callback variable will be called when the service reply
	 * has been received by ROSBridge. It will passed the received data to the callback.
	 * The whole content of the "request" parameter will be send as the "args"
	 * argument of the Service Request
	 */
	bool CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse);

	void MarkAsDisconnected();
	bool Reconnect(UROSIntegrationCore* ROSIntegrationCore);

protected:

	virtual FString GetDetailedInfoInternal() const override;

	UPROPERTY()
	UROSIntegrationCore* _ROSIntegrationCore;

private:

	struct State
	{
		bool Connected;
		bool Advertised;
	} _State;

	// Helper to keep track of self-destruction for async functions
	TSharedPtr<UService, ESPMode::ThreadSafe> _SelfPtr;

	// PIMPL
	class Impl;
	Impl* _Implementation;
};
