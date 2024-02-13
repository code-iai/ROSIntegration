#include "ROSIntegrationGameInstance.h"
#include "ROSIntegrationCore.h"
#include "RI/Topic.h"
#include "RI/Service.h"
#include "ROSTime.h"
#include "rosgraph_msgs/Clock.h"
#include "Misc/App.h"
#include "ROSBridgeParamOverride.h"
#include "Kismet/GameplayStatics.h"
#include "ros_version.h"

// Define ROS_VERSION here
uint8 ROS_VERSION = 1;

void UROSIntegrationGameInstance::Init()
{
	Super::Init();
	ROS_VERSION = ROSVersion;

	// Find AROSBridgeParamOverride actor, if it exists, to override ROS connection parameters
	TArray<AActor*> TempArray;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AROSBridgeParamOverride::StaticClass(), TempArray);
	if (TempArray.Num() > 0)
	{
		AROSBridgeParamOverride* OverrideParams = Cast<AROSBridgeParamOverride>(TempArray[0]);
		if (OverrideParams)
		{
			UE_LOG(LogROS, Display, TEXT("ROSIntegrationGameInstance::Init() - Found AROSBridgeParamOverride to override ROS connection parameters."));
			ROSBridgeServerProtocol = OverrideParams->ROSBridgeServerProtocol;
			ROSBridgeServerHosts = OverrideParams->ROSBridgeServerHosts;
			ROSBridgeServerPorts = OverrideParams->ROSBridgeServerPorts;
			ROS_VERSION = OverrideParams->ROSVersion;
			bConnectToROS = OverrideParams->bConnectToROS;
			bSimulateTime = OverrideParams->bSimulateTime;
			bUseFixedUpdateInterval = OverrideParams->bUseFixedUpdateInterval;
			FixedUpdateInterval = OverrideParams->FixedUpdateInterval;
			bCheckHealth = OverrideParams->bCheckHealth;
			CheckHealthInterval = OverrideParams->CheckHealthInterval;
			ClockTopicName = OverrideParams->ClockTopicName;
		}
	}		

	if (bConnectToROS)
	{
		// Initalize arrays
		if (NumROSBridgeServers == 0)
		{			
			// Add default IP and/or port
			if (ROSBridgeServerHosts.Num() == 0)
			{
				UE_LOG(LogROS, Warning, TEXT("UROSIntegrationGameInstance::Init() - ROSBridgeServerHosts has no hosts provided. Defaulting to 127.0.0.1"));
				ROSBridgeServerHosts.Emplace(TEXT("127.0.0.1"));
			}
			if (ROSBridgeServerPorts.Num() == 0)
			{
				UE_LOG(LogROS, Warning, TEXT("UROSIntegrationGameInstance::Init() - ROSBridgeServerPorts has no ports provided. Defaulting to 9090."));
				ROSBridgeServerPorts.Emplace(9090);
			}

			int32 NumHosts = ROSBridgeServerHosts.Num();
			int32 NumPorts = ROSBridgeServerPorts.Num();
			NumROSBridgeServers = NumHosts < NumPorts ? NumHosts : NumPorts;

			ConnectedToROSBridge.Reset();
			ConnectedToROSBridge.Init(false, NumROSBridgeServers);

			ROSConnections.Reset();
			ROSConnections.Init(nullptr, NumROSBridgeServers);
		}

		for (int32 i = 0; i < NumROSBridgeServers; i++)
			EstablishROSConnection(i);
	}
}

UROSIntegrationCore* UROSIntegrationGameInstance::GetROSConnectionFromID(int32 ID)
{
	if (ID >= 0 && ID < NumROSBridgeServers)
		return ROSConnections[ID];
	else
	{
		UE_LOG(LogROS, Warning, TEXT("UROSIntegrationGameInstance::GetROSConnectionFromID() - rosbridge server ID %i does not exist, returning ID 0 instead"), ID);
		return ROSConnections[0];
	}
}

void UROSIntegrationGameInstance::EstablishROSConnection(int32 ID)
{
	if (!bConnectToROS)
		return;

	if (ID >= 0 && ID < NumROSBridgeServers && !ConnectedToROSBridge[ID])
	{
		// Comment from @tsender:
		// This was from the old version, but I don't think it ever did anything. I'm leaving it commented in case someone thinks its actually useful. 
		// If this is reintegrated, it will need to be in an array format though.
		// bool resLock = initMutex_.TryLock(); 
		// if (!resLock)
		// {
		// 	UE_LOG(LogROS, Display, TEXT("UROSIntegrationGameInstance::Init() - already connection to ROS bridge!"));
		// 	return; // EXIT POINT!
		// }
		// FLocker locker(&initMutex_);
		
		UE_LOG(LogROS, Display, TEXT("UROSIntegrationGameInstance::Init() - connecting to rosbridge %s:%u (ID %i)..."), *ROSBridgeServerHosts[ID], ROSBridgeServerPorts[ID], ID);

		// Destroy old pointer
		if (ROSConnections[ID])
		{
			UROSIntegrationCore* oldRosCore = ROSConnections[ID];
			ROSConnections[ID] = nullptr;
			oldRosCore->ConditionalBeginDestroy();
		}

		ROSConnections[ID] = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass());
		ConnectedToROSBridge[ID] = ROSConnections[ID]->Init(ROSBridgeServerProtocol, ROSBridgeServerHosts[ID], ROSBridgeServerPorts[ID]);

		if (!bTimerSet)
		{
			bTimerSet = true; 
			GetTimerManager().SetTimer(TimerHandle_CheckHealth, this, &UROSIntegrationGameInstance::CheckROSBridgeHealth,
				CheckHealthInterval, true, std::max(5.0f, CheckHealthInterval));
		}

		if (ConnectedToROSBridge[ID])
		{
			UE_LOG(LogROS, Display, TEXT("Successfully connected to rosbridge %s:%u (ID %i)."), *ROSBridgeServerHosts[ID], ROSBridgeServerPorts[ID], ID);
			
			// Only init spawn manager for ROSBridge connection 0
			if (ID == 0)
			{
				ROSIntegrationCore = ROSConnections[0]; // Set this for legacy purposes so others can still use this variable
				UWorld* CurrentWorld = GetWorld();
				if (CurrentWorld)
				{
					ROSConnections[0]->SetWorld(CurrentWorld);
					ROSConnections[0]->InitSpawnManager();
				}
				else
					UE_LOG(LogROS, Display, TEXT("World not available in UROSIntegrationGameInstance::Init()!"));
			}
		}
		else
			UE_LOG(LogROS, Error, TEXT("Failed to connect to server %s:%u. Please make sure that your rosbridge is running."), *ROSBridgeServerHosts[ID], ROSBridgeServerPorts[ID]);

		if (ID == 0)
			FROSTime::SetUseSimTime(false);

		if (bSimulateTime && ID == 0)
		{
			FApp::SetFixedDeltaTime(FixedUpdateInterval);
			FApp::SetUseFixedTimeStep(bUseFixedUpdateInterval);

			// Tell ROSIntegration to use simulated time
			FROSTime now = FROSTime::Now();
			FROSTime::SetUseSimTime(true);
			FROSTime::SetSimTime(now);

			// Only need to do this once
			if (!bAddedOnWorldTickDelegate)
			{
				FWorldDelegates::OnWorldTickStart.AddUObject(this, &UROSIntegrationGameInstance::OnWorldTickStart);
				bAddedOnWorldTickDelegate = true;

				ClockTopic = NewObject<UTopic>(UTopic::StaticClass());
				ClockTopic->Init(ROSConnections[0], ClockTopicName, FString(TEXT("rosgraph_msgs/Clock")), 3);
				ClockTopic->Advertise();
			}
		}
	}
}

void UROSIntegrationGameInstance::CheckROSBridgeHealth()
{
	if (!bCheckHealth) return; 

	// Check how many healthy connections we have
	int32 NumConnected = 0;
	for (int32 i = 0; i < NumROSBridgeServers; i++)
	{
		if (ConnectedToROSBridge[i] && ROSConnections[i]->IsHealthy())
			NumConnected += 1;
		else if (ConnectedToROSBridge[i])
		{
			ConnectedToROSBridge[i] = false;
			UE_LOG(LogROS, Error, TEXT("Connection to rosbridge %s:%u (ID %i) was interrupted."), *ROSBridgeServerHosts[i], ROSBridgeServerPorts[i], i);
		}
	}

	if (OnROSConnectionStatus.IsBound())
		OnROSConnectionStatus.Broadcast(NumConnected, NumROSBridgeServers-NumConnected); // Notify bound functions how many servers are and are not connected

	if (NumConnected == NumROSBridgeServers)
		return;

	// Reconnect to disconnected rosbridge servers
	for (int32 i = 0; i < NumROSBridgeServers; i++)
	{
		if (!ConnectedToROSBridge[i])
		{
			MarkROSObjectsAsDisconnected(i);
			EstablishROSConnection(i);
			if (ConnectedToROSBridge[i])
				ReconnectToROSObjects(i);
		}
	}
}

void UROSIntegrationGameInstance::ShutdownAllROSObjects()
{
	for (TObjectIterator<UTopic> It; It; ++It)
	{
		UTopic* Topic = *It;
		Topic->Unadvertise(); // Must come before unsubscribe becasue unsubscribe can potentially set _ROSTopic to null
		Topic->Unsubscribe();
		Topic->MarkAsDisconnected();
	}
	for (TObjectIterator<UService> It; It; ++It)
	{
		UService* Service = *It;
		Service->Unadvertise();
		Service->MarkAsDisconnected();   
	}
}

void UROSIntegrationGameInstance::MarkAllROSObjectsAsDisconnected()
{
	for (TObjectIterator<UTopic> It; It; ++It)
	{
		UTopic* Topic = *It;
		Topic->MarkAsDisconnected();  
	}
	for (TObjectIterator<UService> It; It; ++It)
	{
		UService* Service = *It;
		Service->MarkAsDisconnected();   
	}
}

void UROSIntegrationGameInstance::MarkROSObjectsAsDisconnected(int32 ID)
{
	for (TObjectIterator<UTopic> It; It; ++It)
	{
		UTopic* Topic = *It;
		if (Topic->GetROSBridgeHost().Compare(ROSBridgeServerHosts[ID]) == 0 && Topic->GetROSBridgePort() == ROSBridgeServerPorts[ID])
			Topic->MarkAsDisconnected();
	}
	for (TObjectIterator<UService> It; It; ++It)
	{
		UService* Service = *It;
		if (Service->GetROSBridgeHost().Compare(ROSBridgeServerHosts[ID]) == 0 && Service->GetROSBridgePort() == ROSBridgeServerPorts[ID])
			Service->MarkAsDisconnected();
	}
}

void UROSIntegrationGameInstance::ReconnectToROSObjects(int32 ID)
{
	for (TObjectIterator<UTopic> It; It; ++It)
	{
		UTopic* Topic = *It;
		if (Topic->GetROSBridgeHost().Compare(ROSBridgeServerHosts[ID]) == 0 && Topic->GetROSBridgePort() == ROSBridgeServerPorts[ID])
		{
			bool success = Topic->Reconnect(ROSConnections[ID]);
			if (!success)
			{
				ConnectedToROSBridge[ID] = false;
				UE_LOG(LogROS, Error, TEXT("Unable to re-establish topic %s."), *Topic->GetDetailedInfo());
			}
		}
	}
	for (TObjectIterator<UService> It; It; ++It)
	{
		UService* Service = *It;
		if (Service->GetROSBridgeHost().Compare(ROSBridgeServerHosts[ID]) == 0 && Service->GetROSBridgePort() == ROSBridgeServerPorts[ID])
		{
			bool success = Service->Reconnect(ROSConnections[ID]);
			if (!success)
			{
				ConnectedToROSBridge[ID] = false;
				UE_LOG(LogROS, Error, TEXT("Unable to re-establish service %s."), *Service->GetDetailedInfo());
			}
		}
	}
}

// N.B.: from log, first comes Shutdown() and then BeginDestroy()
void UROSIntegrationGameInstance::Shutdown()
{
	UE_LOG(LogROS, Display, TEXT("ROS Game Instance - shutdown start"));
	if (bConnectToROS)
	{
		if(bTimerSet) GetTimerManager().ClearTimer(TimerHandle_CheckHealth);

		if (bSimulateTime)
		{
			FWorldDelegates::OnWorldTickStart.RemoveAll(this);
		}

		ShutdownAllROSObjects(); // Stop all ROS objects from advertising, publishing, and subscribing
		MarkAllROSObjectsAsDisconnected(); // Moved here from UROSIntegrationGameInstance::BeginDestroy()

		ConnectedToROSBridge.Reset();

		UE_LOG(LogROS, Display, TEXT("ROS Game Instance - shutdown done"));
	}
	Super::Shutdown();
}

void UROSIntegrationGameInstance::BeginDestroy()
{
	// tell everyone (Topics, Services, etc.) they should stop any interaction with ROS.
	if (bConnectToROS) 
	{
		UE_LOG(LogROS, Display, TEXT("ROS Game Instance - begin destroy - start"));

		//MarkAllROSObjectsAsDisconnected();  // moved in UROSIntegrationGameInstance::Shutdown()

		//ROSIntegrationCore->ConditionalBeginDestroy();
		//ROSIntegrationCore = nullptr; 

		//ClockTopic->ConditionalBeginDestroy(); 
		
		//if (GetWorld()) GetWorld()->ForceGarbageCollection(true);  
	}

	Super::BeginDestroy();

	UE_LOG(LogROS, Display, TEXT("ROS Game Instance - begin destroy - done"));
}

#if ENGINE_MINOR_VERSION > 23 || ENGINE_MAJOR_VERSION >4
void UROSIntegrationGameInstance::OnWorldTickStart(UWorld * World, ELevelTick TickType, float DeltaTime)
#else 
void UROSIntegrationGameInstance::OnWorldTickStart(ELevelTick TickType, float DeltaTime)
#endif
{
	if (bSimulateTime && TickType == ELevelTick::LEVELTICK_TimeOnly)
	{
		FApp::SetFixedDeltaTime(FixedUpdateInterval);
		FApp::SetUseFixedTimeStep(bUseFixedUpdateInterval);

		FROSTime now = FROSTime::Now();

		// advance ROS time
		unsigned long seconds = (unsigned long)DeltaTime;
		unsigned long long nanoseconds = (unsigned long long)(DeltaTime * 1000000000ul);
		unsigned long nanoseconds_only = nanoseconds - (seconds * 1000000000ul);

		now._Sec += seconds;
		now._NSec += nanoseconds_only;

		if (now._NSec >= 1000000000ul)
		{
			now._Sec += 1;
			now._NSec -= 1000000000ul;
		}

		// internal update for ROSIntegration
		FROSTime::SetSimTime(now);

		// send /clock topic to let everyone know what time it is...
		TSharedPtr<ROSMessages::rosgraph_msgs::Clock> ClockMessage(new ROSMessages::rosgraph_msgs::Clock(now));
		ClockTopic->Publish(ClockMessage);
	}
}

