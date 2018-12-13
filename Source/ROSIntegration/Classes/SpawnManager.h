#pragma once

#include <iostream>

#include <Tickable.h>
#include "SpawnableObject.h"
#include "SpawnObjectMessage.h"
#include <EngineUtils.h>

#include "SpawnManager.generated.h"

/**
 * Seperate Spawning Class since received ROSBridge Messages will
 * not have access to the GameThread directly.
 * However, this is necessary in order to use SpawnActor<T>
 */
UCLASS()
class ROSINTEGRATION_API USpawnManager : public UObject, public FTickableGameObject
{
	GENERATED_BODY()
public:
	USpawnManager();
	~USpawnManager();

	void Tick(float DeltaTime) override;
	bool IsTickable() const override;
	bool IsTickableInEditor() const override;
	bool IsTickableWhenPaused() const override;
	TStatId GetStatId() const override;

	UWorld* GetWorld() const override;

	bool _TickingActive = false;

	bool _TestSpawn = false;

	UWorld* _World;


	// Warning:
	// Enqueued Elements will be deleted after the object has been spawned!
	//TQueue<SpawnObjectMessage*, EQueueMode::Mpsc> _SpawnObjectMessageQueue; // Produces Segfaults :(
	//UPROPERTY()
	TQueue<SpawnObjectMessage, EQueueMode::Spsc> _SpawnObjectMessageQueue;
};
