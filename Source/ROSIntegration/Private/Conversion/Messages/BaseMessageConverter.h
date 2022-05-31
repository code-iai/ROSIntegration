#pragma once

#include "CoreMinimal.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#endif // PLATFORM_WINDOWS

#include "ROSIntegrationCore.h"
#include "rosbridge2cpp/messages/rosbridge_publish_msg.h"
#include <cstring>
#include <functional>
#include <bson.h>
#include "std_msgs/Header.h"
#include "ROSTime.h"

#include "BaseMessageConverter.generated.h"

UCLASS()
class ROSINTEGRATION_API UBaseMessageConverter: public UObject
{
	GENERATED_BODY()

public:
	UBaseMessageConverter();
	
	UPROPERTY()
	FString _MessageType;

	// For ConvertMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static double GetDoubleFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors=true)
	{
		assert(msg != nullptr);

		double value = rosbridge2cpp::Helper::get_double_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogROS, Error, TEXT("Key %s not present in data"), *Key);
		}
		return value;
	}

	static FString GetFStringFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		assert(msg != nullptr);

		std::string value = rosbridge2cpp::Helper::get_utf8_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogROS, Error, TEXT("Key %s not present in data"), *Key);
		}
		return UTF8_TO_TCHAR(value.c_str());
	}

	static int32 GetInt32FromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		assert(msg != nullptr);

		int32 value = rosbridge2cpp::Helper::get_int32_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogROS, Error, TEXT("Key %s not present in data"), *Key);
		}
		return value;
	}

	static int64 GetInt64FromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		assert(msg != nullptr);

		int64 value = rosbridge2cpp::Helper::get_int64_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogROS, Error, TEXT("Key %s not present in data"), *Key);
		}
		return value;
	}

	static bool GetBoolFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		assert(msg != nullptr);

		bool value = rosbridge2cpp::Helper::get_bool_by_key(TCHAR_TO_UTF8(*Key), *msg, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogROS, Error, TEXT("Key %s not present in data"), *Key);
		}
		return value;
	}

	static const uint8* GetBinaryFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		assert(msg != nullptr);
		uint32_t binary_data_length = 0; // Gets populated by get_binary_by_key()
		const uint8* value = rosbridge2cpp::Helper::get_binary_by_key(TCHAR_TO_UTF8(*Key), *msg, binary_data_length, KeyFound);
		if (!KeyFound && LogOnErrors) {
			UE_LOG(LogROS, Error, TEXT("Key %s not present in data"), *Key);
		}
		return value;
	}

	/// @note Example usage in GetDoubleTArrayFromBSON().
	template<class T>
	static TArray<T> GetTArrayFromBSON(FString Key, bson_t* msg, bool &KeyFound, const std::function<T(FString, bson_t*, bool&)>& keyToT, bool LogOnErrors = true)
	{
		assert(msg != nullptr);

		uint32_t array_size;
		const uint8_t* data = rosbridge2cpp::Helper::get_array_by_key(TCHAR_TO_UTF8(*Key), *msg, array_size, KeyFound);
		if (!KeyFound)
		{
			if (LogOnErrors) {
				UE_LOG(LogROS, Error, TEXT("Key %s not present in data"), *Key);
			}
			return TArray<T>();
		}

		TArray<T> ret;
		bool elemFound = true;
		for (int i = 0; elemFound; ++i)
		{
			FString iKey = Key + "." + FString::FromInt(i);
			elemFound = bson_has_field(msg, TCHAR_TO_UTF8(*iKey));
			if (elemFound)
			{
				T temp = keyToT(iKey, msg, elemFound);
				ret.Add(temp);
			}
		}

		return ret;
	}

	static TArray<double> GetDoubleTArrayFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{

		return GetTArrayFromBSON<double>(Key, msg, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetDoubleFromBSON(subKey, subMsg, subKeyFound, false); }, LogOnErrors);
	}

	static TArray<float> GetFloatTArrayFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		// bson doesn't support float, only double. So we use GetDoubleFromBSON internally
		return GetTArrayFromBSON<float>(Key, msg, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetDoubleFromBSON(subKey, subMsg, subKeyFound, false); }, LogOnErrors);
	}
	
	static TArray<int32> GetInt32TArrayFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		return GetTArrayFromBSON<int32>(Key, msg, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetInt32FromBSON(subKey, subMsg, subKeyFound, false); }, LogOnErrors);
	}

	static TArray<bool> GetBoolTArrayFromBSON(FString Key, bson_t* msg, bool &KeyFound, bool LogOnErrors = true)
	{
		return GetTArrayFromBSON<bool>(Key, msg, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) { return GetBoolFromBSON(subKey, subMsg, subKeyFound, false); }, LogOnErrors);
	}
	
	template<class T>
	static void _bson_append_tarray(bson_t *b, const char *key, const TArray<T>& tarray, const std::function<void(bson_t*, const char*, const T&)>& appendT)
	{
		bson_t arr;
		const char *element_key;
		char str[16];
		BSON_APPEND_ARRAY_BEGIN(b, key, &arr);
		for (int i = 0; i != tarray.Num(); ++i)
		{
			bson_uint32_to_string(i, &element_key, str, sizeof str);
			appendT(&arr, element_key, tarray[i]);
		}
		bson_append_array_end(b, &arr);
	}

	// Helper function to append a TArray<double> to a bson_t
	static void _bson_append_double_tarray(bson_t *b, const char *key, const TArray<double>& tarray)
	{
		_bson_append_tarray<double>(b, key, tarray, [](bson_t *subb, const char *subKey, double d) { BSON_APPEND_DOUBLE(subb, subKey, d); });
	}

	// Helper function to append a TArray<float> to a bson_t
	static void _bson_append_float_tarray(bson_t *b, const char *key, const TArray<float>& tarray)
	{
		// float -> double doesn't loose precision
		_bson_append_double_tarray(b, key, (TArray<double>)tarray);
	}

	// Helper function to append a TArray<uint8> to a bson_t
	static void _bson_append_uint8_tarray(bson_t *b, const char *key, const TArray<uint8>& tarray)
	{
		// uint8 -> uint32 doesn't loose precision
		_bson_append_uint32_tarray(b, key, (TArray<uint32>)tarray);
	}

	// Helper function to append a TArray<int32> to a bson_t
	static void _bson_append_int32_tarray(bson_t *b, const char *key, const TArray<int32>& tarray)
	{
		_bson_append_tarray<int32>(b, key, tarray, [](bson_t *subb, const char *subKey, int32 i) { BSON_APPEND_INT32(subb, subKey, i); });
	}
	
	// Helper function to append a TArray<uint32> to a bson_t
	static void _bson_append_uint32_tarray(bson_t *b, const char *key, const TArray<uint32>& tarray)
	{
		_bson_append_tarray<uint32>(b, key, tarray, [](bson_t *subb, const char *subKey, uint32 i) { BSON_APPEND_INT32(subb, subKey, i); });
	}
	
	// Helper function to append a TArray<bool> to a bson_t
	static void _bson_append_bool_tarray(bson_t *b, const char *key, const TArray<bool>& tarray)
	{
		_bson_append_tarray<bool>(b, key, tarray, [](bson_t *subb, const char *subKey, bool i) { BSON_APPEND_BOOL(subb, subKey, i); });
	}

	static bool _bson_extract_child_ros_time(bson_t *b, FString key, FROSTime *time, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		time->_Sec = GetInt32FromBSON(key + ".secs", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		time->_NSec = GetInt32FromBSON(key + ".nsecs", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_ros_time(bson_t *b, const char* key, const FROSTime* time)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		BSON_APPEND_INT32(&child, "secs", time->_Sec);
		BSON_APPEND_INT32(&child, "nsecs", time->_NSec);
		bson_append_document_end(b, &child);
	}
};