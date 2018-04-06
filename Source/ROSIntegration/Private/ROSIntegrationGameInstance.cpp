#include "ROSIntegrationGameInstance.h"
#include "std_msgs/String.h"
#include "bson.h" 


void UROSIntegrationGameInstance::Init()
{
	if (bConnectToROS)
	{
        ROSIntegrationCore = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass());
		bIsConnected = ROSIntegrationCore->Init(ROSBridgeServerHost, ROSBridgeServerPort);

		if (bIsConnected)
		{
			UWorld* CurrentWorld = GetWorld();
			if (CurrentWorld) 
            {
                ROSIntegrationCore->SetWorld(CurrentWorld);
                ROSIntegrationCore->InitSpawnManager();
			}
			else 
            {
				UE_LOG(LogROS, Error, TEXT("World not available in UROSIntegrationGameInstance::Init()!"));
			}

            GetTimerManager().SetTimer(TimerHandle_CheckHealth, this, &UROSIntegrationGameInstance::CheckROSBridgeHealth, 1.0f, true, 5.0f);
		}
	}
}

void UROSIntegrationGameInstance::CheckROSBridgeHealth()
{
    if (ROSIntegrationCore->IsHealthy())
    {
        return;
    }

    bIsConnected = false;
    Init();

    if(bConnectToROS && !bIsConnected)
    {
        return; // Let timer call this method again to retry connection attempt
    }

    // TODO: tell everyone (Topics, Services, etc.) they lost connection and need to reconnect (subscribe and advertise)
}

void UROSIntegrationGameInstance::Shutdown() 
{
    GetTimerManager().ClearTimer(TimerHandle_CheckHealth);
}

void UROSIntegrationGameInstance::BeginDestroy() 
{
	Super::BeginDestroy();
}