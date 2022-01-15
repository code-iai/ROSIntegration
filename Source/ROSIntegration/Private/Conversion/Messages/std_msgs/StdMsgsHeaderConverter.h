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
	static bool _bson_extract_child_header(bson_t *b, FString key, ROSMessages::std_msgs::Header *h, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		
		// TODO Check if rosbridge sends UINT64 or INT32 (there is no uint32 in bson)
		h->seq = GetInt32FromBSON(key + ".seq", b, KeyFound, LogOnErrors);		if (!KeyFound) return false;
		int32 Sec = GetInt32FromBSON(key + ".stamp.secs", b, KeyFound, LogOnErrors);  if (!KeyFound) return false;
		int32 NSec = GetInt32FromBSON(key + ".stamp.nsecs", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		h->frame_id = GetFStringFromBSON(key + ".frame_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		h->time = FROSTime(Sec, NSec);

		return true;
	}

	static void _bson_append_child_header(bson_t* b, const char* key, const ROSMessages::std_msgs::Header* header)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_header(&child, header);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_header(bson_t *b, const ROSMessages::std_msgs::Header *header)
	{
		BSON_APPEND_INT32(b, "seq", header->seq);

		bson_t stamp;
		BSON_APPEND_DOCUMENT_BEGIN(b, "stamp", &stamp);
		BSON_APPEND_INT32(&stamp, "secs", header->time._Sec);
		BSON_APPEND_INT32(&stamp, "nsecs", header->time._NSec);
		bson_append_document_end(b, &stamp);

		BSON_APPEND_UTF8(b, "frame_id", TCHAR_TO_UTF8(*header->frame_id));
	}
};
