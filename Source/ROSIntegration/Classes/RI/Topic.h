#pragma once

#include <functional>
#include <memory>
#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
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
	PoseStamped = 2,
	TwistStamped = 3,
	Twist = 4,
	HomePosition = 5
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

	void MarkAsDisconnected();
	bool Reconnect(UROSIntegrationCore* ROSIntegrationCore);
	
	bool IsAdvertising();

protected:

	virtual FString GetDetailedInfoInternal() const override;

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnConstruct();

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnStringMessage(const FString& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnFloat32Message(const float& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnPoseStampedMessage(const FVector& Position, const FRotator& Rotation);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnTwistStampedMessage(const FVector& Linear, const FVector& Angular);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnTwistMessage(const FVector& Linear, const FVector& Angular);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnHomePositionMessage(const FVector& Geo, const FVector& Position, const FRotator& Orientation, const FVector& Approach);

	UPROPERTY()
	UROSIntegrationCore* _ROSIntegrationCore = nullptr;

private:

	struct State
	{
		bool Connected;
		bool Advertised;
		bool Subscribed;
		bool Blueprint;
		EMessageType BlueprintMessageType;
	} _State;


	UFUNCTION(BlueprintCallable, Category = "ROS|Topic")
	void Init(const FString& TopicName, EMessageType MessageType, int32 QueueSize = 1);

	/**
	 * Subscribe to the given topic
	 */
	UFUNCTION(BlueprintCallable, Category = "ROS|Topic")
	bool Subscribe();

	UFUNCTION(BlueprintCallable, Category = "ROS|Topic")
	bool PublishStringMessage(const FString& Message);

	UFUNCTION(BlueprintCallable, Category = "ROS|Topic")
	bool PublishTwistStampedMessage(const FVector& Linear, const FVector& Angular);

	UFUNCTION(BlueprintCallable, Category = "ROS|Topic")
	bool PublishTwistMessage(const FVector& Linear, const FVector& Angular);

	// Helper to keep track of self-destruction for async functions
	TSharedPtr<UTopic, ESPMode::ThreadSafe> _SelfPtr;

	// PIMPL
	class Impl;
	Impl *_Implementation = nullptr;
};
