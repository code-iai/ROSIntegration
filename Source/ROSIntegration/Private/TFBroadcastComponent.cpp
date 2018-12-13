#include "TFBroadcastComponent.h"

#include "ROSIntegrationGameInstance.h"
#include "tf2_msgs/TFMessage.h"
#include "ROSTime.h"

// Sets default values for this component's properties
UTFBroadcastComponent::UTFBroadcastComponent()
: ComponentActive(true)
, FrameRate(1)
, CoordsRelativeTo(ECoordinateType::COORDTYPE_WORLD)
, ParentFrameName(TEXT("/world"))
, ThisFrameName(TEXT("/tfbroadcast_default"))
, UseParentActorLabelAsParentFrame(true)
, UseActorLabelAsFrame(true)
, FrameTime(1.0f / FrameRate)
, TimePassed(0)
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these
	// features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
}

// Called when the game starts
void UTFBroadcastComponent::BeginPlay()
{
	Super::BeginPlay();

	assert(GetOwner());

	_TFTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* ROSInstance = Cast<UROSIntegrationGameInstance>(GetOwner()->GetGameInstance());
	_TFTopic->Init(ROSInstance->ROSIntegrationCore, TEXT("/tf"), TEXT("tf2_msgs/TFMessage"));
}

AActor* UTFBroadcastComponent::GetParentActor()
{
	auto RootComponent = GetOwner()->GetRootComponent();
	assert(RootComponent);
	if (!(RootComponent->GetAttachParent()))
		return nullptr;

	return RootComponent->GetAttachParent()->GetOwner();
}

// Called every frame
void UTFBroadcastComponent::TickComponent(float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Check for framerate
	TimePassed += DeltaTime;
	if (TimePassed < FrameTime) {
		return;
	}
	TimePassed -= FrameTime;

	TickCounter++;

	bool GlobalSettingTFBroadcastEnabled = false;

	//auto World = GetWorld();
	//if (World) {
	//	auto WorldSettings = World->GetWorldSettings();
	//	if (WorldSettings) {
	//		AMyWorldSettings* MyWorldSettings = Cast<AMyWorldSettings>(WorldSettings);
	//		GlobalSettingTFBroadcastEnabled = MyWorldSettings->bEnableTFBroadcast;
	//		//			OUT_INFO(TEXT("MyWorldSettings::bEnableTFBroadcast is %s"), GlobalSettingTFBroadcastEnabled ? TEXT("True") : TEXT("False"));
	//	}
	//	else {
	//		OUT_INFO(TEXT("Failed to GetWorldSettings() - Can't access WorldSettings"));
	//	}
	//}
	//else {
	//	OUT_INFO(TEXT("Failed to GetWorld() - Can't access WorldSettings"));
	//}

	//	OUT_INFO(TEXT("Owner Loc: %s"), *(GetOwner()->GetActorLocation().ToString()));
	TickCounter = 0;

	// Skip execution when TF is deactivated globally
	//if (!GlobalSettingTFBroadcastEnabled) return;

	if (!ComponentActive) return;

	assert(GetOwner() != nullptr);

	// Setup the Frame Names
	// The frame name of this component/actor
	FString CurrentThisFrameName = ThisFrameName;
#if WITH_EDITOR
	if (UseActorLabelAsFrame)
	{
		CurrentThisFrameName = GetOwner()->GetActorLabel();
	}
#endif // WITH_EDITOR

	// The frame name of the parent
	FString CurrentParentFrameName = ParentFrameName;

#if WITH_EDITOR
	// Lookup the parent of this frame in hierarchy
	if (UseParentActorLabelAsParentFrame)
	{
		AActor* ParentActor = GetParentActor();
		if (ParentActor)
		{
			CurrentParentFrameName = ParentActor->GetActorLabel();
			// Force set the CoordsRelativeTo Variable to RELATIVE
			// Please make sure that the child has set it's transformation to 'relative'
			CoordsRelativeTo = ECoordinateType::COORDTYPE_RELATIVE;
		} else {
			UE_LOG(LogROS, Error, TEXT("[TFBroadcast] UseParentActorLabelAsParentFrame==true and No Parent Component on %s - Add a parent actor or deactivate UseParentActorLabelAsParentFrame"), *(GetOwner()->GetActorLabel()));
		}
	}
#endif // WITH_EDITOR

	FVector ActorTranslation;
	FQuat ActorRotation;

	if (CoordsRelativeTo == ECoordinateType::COORDTYPE_RELATIVE) {
		AActor* ParentActor = GetParentActor();
		if (!ParentActor) {
#if WITH_EDITOR
			UE_LOG(LogROS, Error, TEXT("[TFBroadcast] CoordsRelativeTo == ECoordinateType::COORDTYPE_RELATIVE and No Parent Component on %s - Add a parent actor or use world coordinates. Skipping TF Broadcast"), *(GetOwner()->GetActorLabel()));
#else
			UE_LOG(LogROS, Error, TEXT("[TFBroadcast] CoordsRelativeTo == ECoordinateType::COORDTYPE_RELATIVE and No Parent Component - Add a parent actor or use world coordinates. Skipping TF Broadcast"));
#endif // WITH_EDITOR
			return;
		}
		FTransform ThisTransformInWorldCoordinates = GetOwner()->GetRootComponent()->GetComponentTransform();
		FTransform ParentTransformInWorldCoordinates = ParentActor->GetRootComponent()->GetComponentTransform();
		FTransform RelativeTransform = ThisTransformInWorldCoordinates.GetRelativeTransform(ParentTransformInWorldCoordinates);
		ActorTranslation = RelativeTransform.GetLocation();
		ActorRotation = RelativeTransform.GetRotation();
	} else {
		ActorTranslation = GetOwner()->GetActorLocation();
		ActorRotation = GetOwner()->GetActorQuat();
	}

	// Convert to meters and ROS coordinate system
	double TranslationX = ActorTranslation.X / 100.0f;
	double TranslationY = -ActorTranslation.Y / 100.0f;
	double TranslationZ = ActorTranslation.Z / 100.0f;
	double RotationX = -ActorRotation.X;
	double RotationY = ActorRotation.Y;
	double RotationZ = -ActorRotation.Z;
	double RotationW = ActorRotation.W;

	FROSTime time = FROSTime::Now();

	TSharedPtr<ROSMessages::tf2_msgs::TFMessage> TFMessage(new ROSMessages::tf2_msgs::TFMessage());
	ROSMessages::geometry_msgs::TransformStamped TransformStamped;
	TransformStamped.header.seq = 0;
	TransformStamped.header.time = time;
	TransformStamped.header.frame_id = CurrentParentFrameName;
	TransformStamped.child_frame_id = CurrentThisFrameName;
	TransformStamped.transform.translation.x = TranslationX;
	TransformStamped.transform.translation.y = TranslationY;
	TransformStamped.transform.translation.z = TranslationZ;
	TransformStamped.transform.rotation.x = RotationX;
	TransformStamped.transform.rotation.y = RotationY;
	TransformStamped.transform.rotation.z = RotationZ;
	TransformStamped.transform.rotation.w = RotationW;

	TFMessage->transforms.Add(TransformStamped);

	_TFTopic->Publish(TFMessage);

	/*rosbridge2cpp::ROSTime time = rosbridge2cpp::ROSTime::now();

	bson_t *transform = BCON_NEW(
		"transforms",
		"[",
		"{",
		"header", "{",
		"seq", BCON_INT32(0),
		"stamp", "{",
		"secs", BCON_INT32(time.sec_),
		"nsecs", BCON_INT32(time.nsec_),
		"}",
		"frame_id", BCON_UTF8(TCHAR_TO_ANSI(*CurrentParentFrameName)),
		"}",
		"child_frame_id", BCON_UTF8(TCHAR_TO_ANSI(*CurrentThisFrameName)),
		"transform", "{",
		"translation", "{",
		"x", BCON_DOUBLE(TranslationX),
		"y", BCON_DOUBLE(TranslationY),
		"z", BCON_DOUBLE(TranslationZ),
		"}",
		"rotation", "{",
		"x", BCON_DOUBLE(RotationX),
		"y", BCON_DOUBLE(RotationY),
		"z", BCON_DOUBLE(RotationZ),
		"w", BCON_DOUBLE(RotationW),
		"}",
		"}",
		"}",
		"]"
	);

	_tf->SendTransform(*transform);
	*/
}

void UTFBroadcastComponent::SetFramerate(const float _FrameRate)
{
	FrameRate = _FrameRate;
	FrameTime = 1.0f / _FrameRate;
	TimePassed = 0;
}
