// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/Vector3.h"
#include "GeometryMsgsVector3Converter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsVector3Converter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static void _bson_append_child_vector3(bson_t *b, const char *key, ROSMessages::geometry_msgs::Vector3 *v3)
	{
		bson_t vec;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &vec);
		_bson_append_vector3(&vec, v3);
		bson_append_document_end(b, &vec);
	}


	static void _bson_append_vector3(bson_t *b, ROSMessages::geometry_msgs::Vector3 *v3)
	{
		BSON_APPEND_DOUBLE(b, "x", v3->x);
		BSON_APPEND_DOUBLE(b, "y", v3->y);
		BSON_APPEND_DOUBLE(b, "z", v3->z);
	}

};

