// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsTwistConverter.h"
#include "GeometryMsgsTwistWithCovarianceConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UGeometryMsgsTwistWithCovarianceConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_twist_with_covariance(bson_t *b, FString key, ROSMessages::geometry_msgs::TwistWithCovariance *p)
	{
		bool KeyFound = false;

		KeyFound = UGeometryMsgsTwistConverter::_bson_extract_child_twist(b, key + ".twist", &p->twist);
		if (!KeyFound)
			return false;

		p->covariance = GetDoubleTArrayFromBSON(key + ".covariance", b, KeyFound);
		if (!KeyFound || p->covariance.Num() != 36)
			return false;

		return true;
	}

	static void _bson_append_child_twist_with_covariance(bson_t *b, const char *key, ROSMessages::geometry_msgs::TwistWithCovariance *t)
	{
		bson_t twist;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &twist);
		_bson_append_twist_with_covariance(&twist, t);
		bson_append_document_end(b, &twist);
	}

	static void _bson_append_twist_with_covariance(bson_t *b, ROSMessages::geometry_msgs::TwistWithCovariance *t)
	{
		UGeometryMsgsTwistConverter::_bson_append_child_twist(b, "twist", &(t->twist));
		_bson_append_double_tarray(b, "covariance", t->covariance);
	}
};

