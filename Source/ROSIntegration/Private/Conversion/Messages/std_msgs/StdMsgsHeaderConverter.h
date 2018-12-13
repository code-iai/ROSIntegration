#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "Conversion/Messages/BaseMessageConverter.h"

#include "StdMsgsHeaderConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsHeaderConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	// Helper function to extract a child-std_msgs/Header from a bson_t
	static bool _bson_extract_child_header(bson_t *b, FString key, ROSMessages::std_msgs::Header *h)
	{
		bool KeyFound = false;

		// TODO Check if rosbridge sends UINT64 or INT32 (there is no uint32 in bson)
		h->seq =	  GetInt32FromBSON(key + ".seq", b, KeyFound);		if (!KeyFound) return false;
		int32 Sec =   GetInt32FromBSON(key + ".stamp.secs", b, KeyFound);  if (!KeyFound) return false;
		int32 NSec =  GetInt32FromBSON(key + ".stamp.nsecs", b, KeyFound); if (!KeyFound) return false;
		h->frame_id = GetFStringFromBSON(key + ".frame_id", b, KeyFound); if (!KeyFound) return false;
		h->time = FROSTime(Sec, NSec);

		return true;
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
