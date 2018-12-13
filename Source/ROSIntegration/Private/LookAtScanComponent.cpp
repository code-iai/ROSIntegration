#include "LookAtScanComponent.h"


// Sets default values for this component's properties
ULookAtScanComponent::ULookAtScanComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
}


// Called when the game starts
void ULookAtScanComponent::BeginPlay()
{
	Super::BeginPlay();
	// Read current parameters - They might have been changed in the editor
	CurrentOffsetY = MinOffSetY;
	CurrentOffsetZ = MinOffSetZ;

	AActor *OwningActor = GetOwner();
	if (!OwningActor) {
		UE_LOG(LogROS, Warning, TEXT("[ULookAtScanComponent] Actor is nullptr in BeginPlay()"));
		return;
	}

	ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* ROSInstance = Cast<UROSIntegrationGameInstance>(OwningActor->GetGameInstance());
	ExampleTopic->Init(ROSInstance->ROSIntegrationCore, CommandTopic, TEXT("std_msgs/String"));
	ExampleTopic->Subscribe(SubscribeCallback);
}


// Called every frame
void ULookAtScanComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	//FVector TargetActorLocation(0, 0, 90);

	//GetOwner()->SetActorLocationAndRotation(Position, Orientation);

	AActor *OwningActor = GetOwner();
	if (!OwningActor) {
		UE_LOG(LogROS, Warning, TEXT("[ULookAtScanningComponent] Actor is nullptr"));
		return;
	}

	FVector ThisActorLocation = OwningActor->GetActorLocation();
	FRotator ThisActorRotation = OwningActor->GetActorRotation();

	//UE_LOG(LogROS, Warning, TEXT("Actor is now at %f, %f, %f"), ThisActorLocation.X, ThisActorLocation.Y, ThisActorLocation.Z)
	//UE_LOG(LogROS, Warning, TEXT("Rotation is now at %f, %f, %f"), ThisActorRotation.Roll, ThisActorRotation.Pitch, ThisActorRotation.Yaw);
	//UE_LOG(LogROS, Warning, TEXT("max z offsets %f, %f"), MinOffSetZ, MaxOffSetZ);
	//UE_LOG(LogROS, Warning, TEXT("offsets %f, %f"), CurrentOffsetY, CurrentOffsetZ);

	// Do movement
	if (TranslationActive) {
		// Continue with the translations if we don't have to wait for a ping
		// or we need to wait for a ping AND it's present
		if (!WaitForTopicPingBeforeIncrement || (WaitForTopicPingBeforeIncrement && DoNextMovement)) {
			OwningActor->SetActorLocation(FVector(TargetActorLocation.X + DistanceX, TargetActorLocation.Y + CurrentOffsetY, TargetActorLocation.Z + CurrentOffsetZ));

			if (WaitForTopicPingBeforeIncrement)
				DoNextMovement = false; // Reset to wait for the next ping

			CurrentOffsetY += YStepSize;

			if (CurrentOffsetZ >= MaxOffSetZ) { // we're done if we hit Z max
				UE_LOG(LogROS, Warning, TEXT("Z offset is over max %f >= %f"), CurrentOffsetZ, MaxOffSetZ);
				TranslationActive = false; // Stop execution
				return;
			}

			if (CurrentOffsetY >= MaxOffSetY) {
				CurrentOffsetZ += ZStepSize; // Go one "row" higher
				CurrentOffsetY = MinOffSetY;
			}

			// Re-get transform
			ThisActorLocation = OwningActor->GetActorLocation();
		}
	}

	FVector TargetDirectionVector = TargetActorLocation - ThisActorLocation;
	TargetDirectionVector.Normalize();

	FRotator NewRotationRPY = FRotationMatrix::MakeFromX(TargetDirectionVector).Rotator();

	OwningActor->SetActorRotation(NewRotationRPY.Quaternion());
}
