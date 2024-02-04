#include "ROSIntegrationGameInstance.h"
#include "RI/Topic.h"
#include "RI/Service.h"
#include "ROSTime.h"
#include "rosgraph_msgs/Clock.h"
#include "Misc/App.h"
#include "ROSBridgeParamOverride.h"
#include "Kismet/GameplayStatics.h"


void UROSIntegrationGameInstance::Init()
{
	Super::Init();

	if (bConnectToROS)
	{
		bool resLock = initMutex_.TryLock(); 
		if (!resLock)
		{
			UE_LOG(LogROS, Display, TEXT("UROSIntegrationGameInstance::Init() - already connection to ROS bridge!"));
			return; // EXIT POINT!
		}

		FLocker locker(&initMutex_);

		UE_LOG(LogROS, Display, TEXT("UROSIntegrationGameInstance::Init() - connecting to ROS bridge..."));

		FROSTime::SetUseSimTime(false);

		if (ROSIntegrationCore)
		{
			UROSIntegrationCore* oldRosCore = ROSIntegrationCore;
			ROSIntegrationCore = nullptr;
			oldRosCore->ConditionalBeginDestroy();
		}

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
				ROSBridgeServerHost = OverrideParams->ROSBridgeServerHost;
				ROSBridgeServerPort = OverrideParams->ROSBridgeServerPort;
				bConnectToROS = OverrideParams->bConnectToROS;
				bSimulateTime = OverrideParams->bSimulateTime;
				bUseFixedUpdateInterval = OverrideParams->bUseFixedUpdateInterval;
				FixedUpdateInterval = OverrideParams->FixedUpdateInterval;
				bCheckHealth = OverrideParams->bCheckHealth;
				CheckHealthInterval = OverrideParams->CheckHealthInterval;
			}
		}

		ROSIntegrationCore = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass()); // ORIGINAL 
		bIsConnected = ROSIntegrationCore->Init(ROSBridgeServerProtocol, ROSBridgeServerHost, ROSBridgeServerPort);

		if (!bTimerSet)
		{
			bTimerSet = true; 
			GetTimerManager().SetTimer(TimerHandle_CheckHealth, this, &UROSIntegrationGameInstance::CheckROSBridgeHealth,
				CheckHealthInterval, true, std::max(5.0f, CheckHealthInterval));
		}

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
				UE_LOG(LogROS, Display, TEXT("World not available in UROSIntegrationGameInstance::Init()!"));
			}
		}
		else if (!bReconnect)
		{
			UE_LOG(LogROS, Error, TEXT("Failed to connect to server %s:%u. Please make sure that your rosbridge is running."), *ROSBridgeServerHost, ROSBridgeServerPort);
		}

		if (bSimulateTime)
		{
			FApp::SetFixedDeltaTime(FixedUpdateInterval);
			FApp::SetUseFixedTimeStep(bUseFixedUpdateInterval);

			// tell ROSIntegration to use simulated time
			FROSTime now = FROSTime::Now();
			FROSTime::SetUseSimTime(true);
			FROSTime::SetSimTime(now);

			if (!bAddedOnWorldTickDelegate)
			{
				FWorldDelegates::OnWorldTickStart.AddUObject(this, &UROSIntegrationGameInstance::OnWorldTickStart);
				bAddedOnWorldTickDelegate = true;
			}

			ClockTopic = NewObject<UTopic>(UTopic::StaticClass()); // ORIGINAL

			ClockTopic->Init(ROSIntegrationCore, FString(TEXT("/clock")), FString(TEXT("rosgraph_msgs/Clock")), 3);

			ClockTopic->Advertise();
		}
	}
}

void UROSIntegrationGameInstance::CheckROSBridgeHealth()
{
	if (!bCheckHealth) return; 

	if (bIsConnected && ROSIntegrationCore->IsHealthy())
	{
		if (OnROSConnectionStatus.IsBound())
		{
			OnROSConnectionStatus.Broadcast(true); // Notify bound functions that we are connected to rosbridge
		}
		return;
	}

	if (bIsConnected)
	{
		UE_LOG(LogROS, Error, TEXT("Connection to rosbridge %s:%u was interrupted."), *ROSBridgeServerHost, ROSBridgeServerPort);
		if (OnROSConnectionStatus.IsBound())
		{
			OnROSConnectionStatus.Broadcast(false); // Notify bound functions that we lost rosbridge connection
		}
	}

	// reconnect again
	bIsConnected = false;
	bReconnect = true;
	Init();
	bReconnect = false;

	// tell everyone (Topics, Services, etc.) they lost connection and should stop any interaction with ROS for now.
	MarkAllROSObjectsAsDisconnected();

	if (!bIsConnected)
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
		for (TObjectIterator<UService> It; It; ++It)
		{
			UService* Service = *It;

			bool success = Service->Reconnect(ROSIntegrationCore);
			if (!success)
			{
				bIsConnected = false;
				UE_LOG(LogROS, Error, TEXT("Unable to re-establish service %s."), *Service->GetDetailedInfo());
			}
		}
	}

	UE_LOG(LogROS, Display, TEXT("Successfully reconnected to rosbridge %s:%u."), *ROSBridgeServerHost, ROSBridgeServerPort);
}

void UROSIntegrationGameInstance::ShutdownAllROSObjects()
{
	for (TObjectIterator<UTopic> It; It; ++It)
	{
		UTopic* Topic = *It;
		if (bIsConnected)
		{
			Topic->Unadvertise(); // Must come before unsubscribe becasue unsubscribe can potentially set _ROSTopic to null
			Topic->Unsubscribe();
		}
		Topic->MarkAsDisconnected();
	}
	for (TObjectIterator<UService> It; It; ++It)
	{
		UService* Service = *It;
		if (bIsConnected)
		{
			Service->Unadvertise();
		}
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
		MarkAllROSObjectsAsDisconnected(); // moved here from UROSIntegrationGameInstance::BeginDestroy()

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

