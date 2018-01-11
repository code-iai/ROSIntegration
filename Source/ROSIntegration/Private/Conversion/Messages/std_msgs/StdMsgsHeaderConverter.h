// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "StdMsgsHeaderConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsHeaderConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	// Helper function to append a std_msgs/Header to a bson_t as a child document
	static void _bson_append_child_header(bson_t *b, const char *key, ROSMessages::std_msgs::Header *h)
	{
		bson_t hdr;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &hdr);
		_bson_append_header(b, h);
		bson_append_document_end(b, &hdr);
	}

	// Helper function to append a std_msgs/Header to a bson_t
	static void _bson_append_header(bson_t *b, ROSMessages::std_msgs::Header *h)
	{
		bson_t *hdr = BCON_NEW(
			"seq", BCON_INT32(h->seq),
			"stamp", "{",
			"secs", BCON_INT32(h->time._Sec),
			"nsecs", BCON_INT32(h->time._NSec),
			"}",
			"frame_id", BCON_UTF8(TCHAR_TO_ANSI(*h->frame_id))
		);
		BSON_APPEND_DOCUMENT(b, "header", hdr);
	}

};

