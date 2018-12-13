#pragma once

#include <CoreMinimal.h>

/**
 * All the important aspects from visualization_msgs/Marker to spawn some objects in Unreal
 */
class ROSINTEGRATION_API SpawnObjectMessage
{
public:
	enum OBJECT_TYPE {
		ARROW = 0,
		CUBE = 1,
		SPHERE = 2,
		CYLINDER = 3,
		LINE_STRIP = 4,
		LINE_LIST = 5,
		CUBE_LIST = 6,
		SPHERE_LIST = 7,
		POINTS = 8,
		TEXT_VIEW_FACING = 9,
		MESH_RESOURCE = 10,
		TRIANGLE_LIST = 11,
	};

	enum ACTION_TYPE {
		ACTION_ADD = 0,
		ACTION_MODIFY = 0,
		ACTION_DELETE = 2,
		ACTION_DELETEALL = 3
	};

	SpawnObjectMessage();
	~SpawnObjectMessage();

	// An identifier for the object to spawn. Used to modify or delete it with following messages
	int32 _Id;

	// The Type of Entity to spawn. See @SpawnObjectMessage::OBJECT_TYPE
	int32 _Type;

	// 0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall
	int32 _Action;

	struct Pose
	{
		FVector _Position;
		FQuat _Orientation;
	};
	Pose _Pose;

	FVector _Scale;
	FLinearColor _Color;

	// Not implemented
	uint32 _Lifetime;
	// Not implemented
	bool _FrameLocked;

	// TODO Points
	// TODO Colors

	// When using _Type==OBJECT_TYPE::MESH_RESOURCE, this string shall hold the path to the material to apply.
	FString _Text;
	// When using _Type==OBJECT_TYPE::MESH_RESOURCE, this string shall hold the path to the mesh to set.
	FString _MeshResource;
	// TODO MeshUseEmbeddedMaterials
};
