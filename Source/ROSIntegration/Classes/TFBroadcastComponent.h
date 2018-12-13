#pragma once

#include <Components/ActorComponent.h>
#include "RI/Topic.h"

#include "TFBroadcastComponent.generated.h"

UENUM(BlueprintType)
enum class ECoordinateType : uint8
{
	COORDTYPE_RELATIVE	UMETA(DisplayName = "Relative to Parent"),
	COORDTYPE_WORLD	UMETA(DisplayName = "World Coordinates")
};


UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class ROSINTEGRATION_API UTFBroadcastComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UTFBroadcastComponent();

	// Called when the game starts
	virtual void BeginPlay() override;

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Activate this Component by setting this flag to TRUE
	UPROPERTY(EditAnywhere)
	bool ComponentActive;

	// How often shall this frame be published (in Hz)?
	UPROPERTY(EditAnywhere)
	uint32 FrameRate;

	// Sets wether the coordinates of this actor shall be published in world coordinates or relative to the owning Actor
	// If you set 'relative' here, please make sure that the actor of this component has a parent actor.
	UPROPERTY(EditAnywhere)
	ECoordinateType CoordsRelativeTo;

	// Name of the Parentframe. This value will be set in the header of the ROS TF Message.
	UPROPERTY(EditAnywhere)
	FString ParentFrameName;

	// Name of this Frame. This value will be set in the child_frame_id of the ROS TF Message.
	UPROPERTY(EditAnywhere)
	FString ThisFrameName;

	// If ticked, TFComponent will check which Actors owns the Actor with this Component.
	// It will then use the ActorLabel of the Parent as the ParentFrameName.
	// When this mode is activated, the option 'CoordsRelativeTo' will be set to "Relative to Parent" and
	// 'ParentFrameName' will be ignored.
	UPROPERTY(EditAnywhere)
	bool UseParentActorLabelAsParentFrame;

	// If ticked, the ActorLabel of owning Actor will be used. 'ThisFrameName' will be ignored when used.
	UPROPERTY(EditAnywhere)
	bool UseActorLabelAsFrame;

	float FrameTime, TimePassed;

	void SetFramerate(const float _FrameRate);

	UPROPERTY()
	UTopic *_TFTopic;

	// Returns null if no parent has been found
	AActor *GetParentActor();

private:
	uint32 TickCounter = 0;
};
