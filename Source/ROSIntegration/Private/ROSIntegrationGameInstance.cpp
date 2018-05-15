#include "ROSIntegrationGameInstance.h"
#include "std_msgs/String.h"
#include "bson.h" 


void UROSIntegrationGameInstance::Init()
{
	if (bConnectToROS)
	{
        ROSIntegrationCore = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass());
		bIsConnected = ROSIntegrationCore->Init(ROSBridgeServerHost, ROSBridgeServerPort);

        GetTimerManager().SetTimer(TimerHandle_CheckHealth, this, &UROSIntegrationGameInstance::CheckROSBridgeHealth, 1.0f, true, 5.0f);

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
		}
        else if(!bReconnect)
        {
            UE_LOG(LogROS, Error, TEXT("Failed to connect to server %s:%u. Please make sure that your rosbridge is running."), *ROSBridgeServerHost, ROSBridgeServerPort);
        }
	}
}

void UROSIntegrationGameInstance::CheckROSBridgeHealth()
{
    if (bIsConnected && ROSIntegrationCore->IsHealthy())
    {
        return;
    }

    if (bIsConnected)
    {
        UE_LOG(LogROS, Error, TEXT("Connection to rosbridge %s:%u was interrupted."), *ROSBridgeServerHost, ROSBridgeServerPort);
    }

    // reconnect again
    bIsConnected = false;
    bReconnect = true;
    Init();
    bReconnect = false;

    // tell everyone (Topics, Services, etc.) they lost connection and should stop any interaction with ROS for now.
    {
        for (TObjectIterator<UTopic> It; It; ++It)
        {
            UTopic* Topic = *It;

            Topic->MarkAsDisconnected();
        }
        // TODO: also tell Services, Actions, etc.
    }

    if(!bIsConnected)
    {
        return; // Let timer call this method again to retry connection attempt
    }

    // tell everyone (Topics, Services, etc.) they can try to reconnect (subscribe and advertise)
    {
        for (TObjectIterator<UTopic> It; It; ++It)
        {
            UTopic* Topic = *It;

            bool success = Topic->Reconnect(ROSIntegrationCore);
            if (!success)
            {
                bIsConnected = false;
                UE_LOG(LogROS, Error, TEXT("Unable to re-establish topic %s."), *Topic->GetDetailedInfo());
            }
        }
        // TODO: also tell Services, Actions, etc.
    }

    UE_LOG(LogROS, Display, TEXT("Successfully reconnected to rosbridge %s:%u."), *ROSBridgeServerHost, ROSBridgeServerPort);
}

void UROSIntegrationGameInstance::Shutdown() 
{
    GetTimerManager().ClearTimer(TimerHandle_CheckHealth);
}

void UROSIntegrationGameInstance::BeginDestroy() 
{
	Super::BeginDestroy();
}