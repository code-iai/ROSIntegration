#include "ROSIntegrationGameInstance.h"
#include "std_msgs/String.h"
#include "bson.h" 


void UROSIntegrationGameInstance::Init() {
	_Ric = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass());
	_Ric->Init(ROSBridgeServerHost, ROSBridgeServerPort);

	UWorld* CurrentWorld = GetWorld();
	if (CurrentWorld) {
		_Ric->SetWorld(CurrentWorld);
		_Ric->InitSpawnManager();
	} else {
		UE_LOG(LogTemp, Error, TEXT("World not available in UROSIntegrationGameInstance::Init()!"));
	}

}

void UROSIntegrationGameInstance::Shutdown() {
}

void UROSIntegrationGameInstance::BeginDestroy() {
	Super::BeginDestroy();
}