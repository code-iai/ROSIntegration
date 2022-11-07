#pragma once

#include <functional>
#include <memory>
#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Math/Vector.h"
#include "ROSBaseMsg.h"
#include "ROSIntegrationCore.h"

#include "Topic.generated.h"

/**
* @ingroup ROS Message Types
* Which Message type to work with.
* NOTE: UFUNCTIONS in UE4 do not support type 'double' (only UE5 supports this)
*/
UENUM(BlueprintType, Category = "ROS")
enum class EMessageType : uint8
{
	String = 0,
	Float32 = 1,
	Bool = 3,
	Header = 8,
	Int32 = 11,
	Int64 = 12,

	Vector3 = 2,
	Point = 17,
	Pose = 16,
	Quaternion = 18,
	Twist = 19,
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

	// Std msgs

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnStringMessage(const FString& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnFloat32Message(const float& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnBoolMessage(const int& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnInt32Message(const int32& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnInt64Message(const int64& Data);


	// Geometry msgs

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnVector3Message(const FVector& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnPointMessage(const FVector& Data);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnPoseMessage(const FVector& position, const FVector4& orientation);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnQuaternionMessage( const FVector4& quaternion);

	UFUNCTION(BlueprintImplementableEvent, Category = ROS)
	void OnTwistMessage(const FVector& linear, const FVector& angular);

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

	// Helper to keep track of self-destruction for async functions
	TSharedPtr<UTopic, ESPMode::ThreadSafe> _SelfPtr;

	// PIMPL
	class Impl;
	Impl *_Implementation = nullptr;
};
