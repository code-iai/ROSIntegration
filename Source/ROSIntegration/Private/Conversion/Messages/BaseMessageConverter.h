// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "rosbridge2cpp/messages/rosbridge_publish_msg.h"
#include <cstring>
#include "bson.h"
#include "Public/std_msgs/Header.h"
#include "BaseMessageConverter.generated.h"

UCLASS()
class ROSINTEGRATION_API UBaseMessageConverter: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY()
	FString _MessageType;

	//For ConvertMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	double GetDoubleFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors=true) {
		assert(msg != nullptr);

		double value = rosbridge2cpp::Helper::get_double_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogTemp, Error, TEXT("Key %s not present in data"), *Key);
		}
		return value;
	}

	FString GetFStringFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true) {
		assert(msg != nullptr);

		std::string value = rosbridge2cpp::Helper::get_utf8_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogTemp, Error, TEXT("Key %s not present in data"), *Key);
		}
		return UTF8_TO_TCHAR(value.c_str());
	}

	int32 GetInt32FromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true) {
		assert(msg != nullptr);

		int32 value = rosbridge2cpp::Helper::get_int32_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogTemp, Error, TEXT("Key %s not present in data"), *Key);
		}
		return value;
	}

protected:

	// Helper function to append a std_msgs/Header to a bson_t
	static void _bson_append_header(bson_t *b, ROSMessages::std_msgs::Header header)
	{
		bson_t *hdr = BCON_NEW(
			"seq", BCON_INT32(header.seq),
			"stamp", "{",
			"secs", BCON_INT32(header.time._Sec),
			"nsecs", BCON_INT32(header.time._NSec)
		);
		BSON_APPEND_DOCUMENT(b, "header", hdr);
	}

	// Helper function to append a TArray<float> to a bson_t
	static void _bson_append_float_tarray(bson_t *b, const char *key, TArray<float> tarray)
	{
		// float -> double doesn't loose precision
		_bson_append_double_tarray(b, key, (TArray<double>)tarray);
	}

	// Helper function to append a TArray<double> to a bson_t
	static void _bson_append_double_tarray(bson_t *b, const char *key, TArray<double> tarray)
	{
		bson_t arr;
		const char *element_key;
		char str[16];
		BSON_APPEND_ARRAY_BEGIN(b, key, &arr);
		for (int i = 0; i != tarray.Num(); ++i)
		{
			bson_uint32_to_string(i, &element_key, str, sizeof str);
			BSON_APPEND_DOUBLE(b, element_key, tarray[i]);
		}
		bson_append_array_end(b, &arr);
	}

	// Helper function to append a TArray<uint8> to a bson_t
	static void _bson_append_uint8_tarray(bson_t *b, const char *key, TArray<uint8> tarray)
	{
		// uint8 -> uint32 doesn't loose precision
		_bson_append_uint32_tarray(b, key, (TArray<uint32>)tarray);
	}

	// Helper function to append a TArray<uint32> to a bson_t
	static void _bson_append_uint32_tarray(bson_t *b, const char *key, TArray<uint32> tarray)
	{
		bson_t arr;
		const char *element_key;
		char str[16];
		BSON_APPEND_ARRAY_BEGIN(b, key, &arr);
		for (int i = 0; i != tarray.Num(); ++i)
		{
			bson_uint32_to_string(i, &element_key, str, sizeof str);
			// XXX ajs 10/Jan/2018 Conversion from uint32 to int32 is not safe
			BSON_APPEND_INT32(b, element_key, tarray[i]);
		}
		bson_append_array_end(b, &arr);
	}

};

