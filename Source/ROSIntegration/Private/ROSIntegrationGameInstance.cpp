#include "ROSIntegrationGameInstance.h"
#include "std_msgs/String.h"
#include "bson.h" 


void UROSIntegrationGameInstance::Init() {
	if (bConnectToROS)
	{
		_Ric = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass());
		bIsConnected = _Ric->Init(ROSBridgeServerHost, ROSBridgeServerPort);

		if (bIsConnected)
		{
			UWorld* CurrentWorld = GetWorld();
			if (CurrentWorld) {
				_Ric->SetWorld(CurrentWorld);
				_Ric->InitSpawnManager();
			}
			else {
				UE_LOG(LogROS, Error, TEXT("World not available in UROSIntegrationGameInstance::Init()!"));
			}
		}
	}
}

void UROSIntegrationGameInstance::Shutdown() {
}

void UROSIntegrationGameInstance::BeginDestroy() {
	Super::BeginDestroy();
}