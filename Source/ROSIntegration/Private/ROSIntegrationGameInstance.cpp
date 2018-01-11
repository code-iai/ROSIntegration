#include "ROSIntegrationGameInstance.h"
#include "std_msgs/String.h"
#include "bson.h" 


void UROSIntegrationGameInstance::Init() {
	UE_LOG(LogTemp, Warning, TEXT("Init on GameInstance"));

	_Ric = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass());
	_Ric->Init(ROSBridgeServerHost, ROSBridgeServerPort);

	UWorld* CurrentWorld = GetWorld();
	if (CurrentWorld) {
		UE_LOG(LogTemp, Error, TEXT("World ready in UROSIntegrationGameInstance::Init()!"));
		_Ric->SetWorld(CurrentWorld);
		_Ric->InitSpawnManager();
	} else {
		UE_LOG(LogTemp, Error, TEXT("World not available in UROSIntegrationGameInstance::Init()!"));
	}

}

void UROSIntegrationGameInstance::Shutdown() {
	UE_LOG(LogTemp, Warning, TEXT("Shutdown on GameInstance"));
}

void UROSIntegrationGameInstance::BeginDestroy() {
	UE_LOG(LogTemp, Warning, TEXT("BeginDestroy on GameInstance"));
	Super::BeginDestroy();
}