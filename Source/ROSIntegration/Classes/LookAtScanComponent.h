#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>
#include "RI/Topic.h"
#include "ROSIntegrationGameInstance.h"
#include "std_msgs/String.h"

#include "LookAtScanComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class ROSINTEGRATION_API ULookAtScanComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	ULookAtScanComponent();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

	// This Vector is the target where the owning actor should always look to
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	FVector TargetActorLocation;

	// The ROS Topic where you can send your commands to
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	FString CommandTopic = TEXT("/camera_orbit_topic");

	// Boundary for Y to offset the owning actor
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	float MinOffSetY = -50.0f;
	float CurrentOffsetY = MinOffSetY;

	// Boundary for Y to offset the owning actor
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	float MaxOffSetY = 50.0f;

	// Boundary for Z to offset the owning actor
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	float MinOffSetZ = -50.0f;
	float CurrentOffsetZ = MinOffSetZ;

	// Boundary for Z to offset the owning actor
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	float MaxOffSetZ = 50.0f;

	// Movement step size on the Y axis
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	float YStepSize = 1.0f;

	// Movement step size on the Z axis
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	float ZStepSize = 1.0f;

	// On the first TickComponent() with Translation enabled , the owning Actor will be set to: FVector(TargetActorLocation.X + DistanceX, TargetActorLocation.Y + CurrentOffsetY, TargetActorLocation.Z + CurrentOffsetZ)
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	float DistanceX = 100.0f;

	// If true, the owning Actor will be translated step by step according to the Y/Z offset settings
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	bool TranslationActive = true;

	// If true, the owning Actor will be only translated after a "ping" has been sent to Topic set by CommandTopic
	UPROPERTY(EditAnywhere, Category = "Offset configuration")
	bool WaitForTopicPingBeforeIncrement = false;

	bool DoNextMovement = false;



	std::function<void(TSharedPtr<FROSBaseMsg>)> SubscribeCallback = [this](TSharedPtr<FROSBaseMsg> msg) -> void
	{
		auto Concrete = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
		if (Concrete.IsValid())
		{
			UE_LOG(LogROS, Log, TEXT("[ULookAtScanningComponent] Command was: %s"), *Concrete->_Data);
			FString Command = *(Concrete->_Data);

			if (Command == TEXT("reset")) {
				UE_LOG(LogROS, Log, TEXT("[ULookAtScanningComponent] Resetting procedure"));
				ResetProcedure();
			}
			else if (Command == TEXT("ping")) {
				DoNextMovement = true;
			}
		}
		return;
	};

	UTopic *ExampleTopic;

public:
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void ResetProcedure() {
		TranslationActive = true;
		CurrentOffsetY = MinOffSetY;
		CurrentOffsetZ = MinOffSetZ;
	}
};
