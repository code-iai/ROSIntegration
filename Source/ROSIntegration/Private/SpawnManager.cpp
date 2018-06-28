#include "SpawnManager.h"

USpawnManager::USpawnManager()
{
	std::cout << "Hi from USpawnManager CTOR" << std::endl;
	AddToRoot();
	//	 _SpawnObjectMessageQueue = new TQueue<SpawnObjectMessage, EQueueMode::Spsc>;
}

USpawnManager::~USpawnManager()
{
	RemoveFromRoot();
}

void USpawnManager::Tick(float DeltaTime)
{
	SpawnObjectMessage Message;
	while (_SpawnObjectMessageQueue.Dequeue(Message)) {
		std::cout << "SpawnManager: Handling SpawnObjectMessage" << std::endl;
		std::cout << "Message contains: Action " << Message._Action  << " and Type: " << Message._Type<< std::endl;
		std::cout << "				 " << TCHAR_TO_UTF8(*Message._MeshResource) << std::endl;
		std::cout << "				 " << TCHAR_TO_UTF8(*Message._Text) << std::endl;

		//if (Message._Action == SpawnObjectMessage::ACTION_TYPE::MODIFY) {
		//	std::cout << "SpawnManager: Modify Messages are not supported currently" << std::endl;
		//	continue;
		//}

		// TODO Implement MODIFY correctly. Right now, it will just add another Actor
		if (Message._Action == SpawnObjectMessage::ACTION_TYPE::ACTION_DELETE ||
			Message._Action == SpawnObjectMessage::ACTION_TYPE::ACTION_DELETEALL)
		{
			for (TActorIterator<ASpawnableObject> ActorItr(GetWorld()); ActorItr; ++ActorItr) {
				ASpawnableObject* Obj = *ActorItr;
				if (ActorItr->GetWorld() != this->GetWorld()) {
					std::cout << "Skipping object from other World" << std::endl;
					continue;
				}
				if (Message._Action == SpawnObjectMessage::ACTION_TYPE::ACTION_DELETEALL ||
					(Message._Action == SpawnObjectMessage::ACTION_TYPE::ACTION_DELETE && Obj->Id == Message._Id)) {
					// Delete the actor if ID matches and action == DELETE
					this->GetWorld()->DestroyActor(Obj);
				}
			}
			// DELETE* action handeled. Skip to next item in Queue.
			continue;
		}

		// Only ADD/MODIFY actions should be left at this point
		assert(Message._Action == SpawnObjectMessage::ACTION_TYPE::ACTION_ADD ||
			Message._Action == SpawnObjectMessage::ACTION_TYPE::ACTION_MODIFY);

		if (!(Message._Type == SpawnObjectMessage::OBJECT_TYPE::CUBE ||
			Message._Type == SpawnObjectMessage::OBJECT_TYPE::CYLINDER ||
			Message._Type == SpawnObjectMessage::OBJECT_TYPE::SPHERE ||
			Message._Type == SpawnObjectMessage::OBJECT_TYPE::MESH_RESOURCE)) {
			std::cerr << "Pulled unsupported Object Type from queued SpawnMessage: " << Message._Type << std::endl;
			return;
		}

		// Convert ROS units (metres) to Unreal units (cms)
		// http://www.ros.org/reps/rep-0103.html
		// Fix rotation
		Message._Pose._Position.X = Message._Pose._Position.X * 100.0f;
		Message._Pose._Position.Y = -Message._Pose._Position.Y * 100.0f;
		Message._Pose._Position.Z = Message._Pose._Position.Z * 100.0f;

		Message._Pose._Orientation.X = -Message._Pose._Orientation.X;
		Message._Pose._Orientation.Y = Message._Pose._Orientation.Y;
		Message._Pose._Orientation.Z = -Message._Pose._Orientation.Z;
		Message._Pose._Orientation.W = Message._Pose._Orientation.W;

		// Spawn the first iteration of the object
		FRotator ObjectRotation(Message._Pose._Orientation);

		ASpawnableObject* SpawnedObject = GetWorld()->SpawnActor<ASpawnableObject>(Message._Pose._Position, ObjectRotation);

		assert(SpawnedObject != nullptr);

		// Refine appearance
		switch (Message._Type) {
		case SpawnObjectMessage::OBJECT_TYPE::CUBE:
			SpawnedObject->MeshToAssign = TEXT("/Engine/BasicShapes/Cube");
			SpawnedObject->MaterialToAssign = TEXT("/Engine/BasicShapes/BasicShapeMaterial");
			SpawnedObject->ColorToAssign = Message._Color;
			SpawnedObject->SetStaticMesh();
			SpawnedObject->SetMaterial();
			break;

		case SpawnObjectMessage::OBJECT_TYPE::CYLINDER:
			SpawnedObject->MeshToAssign = TEXT("/Engine/BasicShapes/Cylinder");
			SpawnedObject->MaterialToAssign = TEXT("/Engine/BasicShapes/BasicShapeMaterial");
			SpawnedObject->ColorToAssign = Message._Color;
			SpawnedObject->SetStaticMesh();
			SpawnedObject->SetMaterial();
			break;

		case SpawnObjectMessage::OBJECT_TYPE::SPHERE:
			SpawnedObject->MeshToAssign = TEXT("/Engine/BasicShapes/Sphere");
			SpawnedObject->MaterialToAssign = TEXT("/Engine/BasicShapes/BasicShapeMaterial");
			SpawnedObject->ColorToAssign = Message._Color;
			SpawnedObject->SetStaticMesh();
			SpawnedObject->SetMaterial();
			break;

		case SpawnObjectMessage::OBJECT_TYPE::MESH_RESOURCE:
			//SpawnedObject->MeshToAssign = Message._MeshResource;
			//SpawnedObject->MaterialToAssign = Message._Text;

			SpawnedObject->MeshToAssign = Message._MeshResource;
			SpawnedObject->MaterialToAssign = Message._Text;
			//SpawnedObject->MaterialToAssign = TEXT("/Engine/BasicShapes/BasicShapeMaterial");
			//SpawnedObject->ColorToAssign = Message._Color;
			//SpawnedObject->ActivatePhysics = true;
			SpawnedObject->SetStaticMesh();
			SpawnedObject->SetMaterial();
			break;

		default:
			std::cerr << "Unsupported Object Type in type handling. This should never happen. Check pre-check. Type:"
				<< Message._Type << std::endl;
			break;
		}
		// Set the scale of the object (Actor)
		SpawnedObject->SetActorScale3D(Message._Scale);
		SpawnedObject->Id = Message._Id;

		// Message has been handeled. Delete it
		// delete Message;
	}
}

bool USpawnManager::IsTickable() const
{
	// see also:
	// https:answers.unrealengine.com/questions/49229/ftickablegameobjecttick-is-being-called-in-the-edi.html
	return _TickingActive;
}

bool USpawnManager::IsTickableInEditor() const
{
	return false;
}

bool USpawnManager::IsTickableWhenPaused() const
{
	return false;
}

TStatId USpawnManager::GetStatId() const
{
	return TStatId();
}

UWorld* USpawnManager::GetWorld() const
{
	return _World;
}
