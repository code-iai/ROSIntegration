#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/sensor_msgs/SensorMsgsRegionOfInterestConverter.h"
#include "sensor_msgs/CameraInfo.h"
#include "SensorMsgsCameraInfoConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsCameraInfoConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	USensorMsgsCameraInfoConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_camera_info(bson_t *b, FString key, ROSMessages::sensor_msgs::CameraInfo *msg)
	{
		bool KeyFound = false;

		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->height = GetInt32FromBSON(key + ".height", b, KeyFound); if (!KeyFound) return false;
		msg->width = GetInt32FromBSON(key + ".width", b, KeyFound); if (!KeyFound) return false;
		msg->distortion_model = GetFStringFromBSON(key + ".distortion_model", b, KeyFound); if (!KeyFound) return false;

		msg->D = GetDoubleTArrayFromBSON(key + "d", b, KeyFound); if (!KeyFound) return false;
		msg->K = GetDoubleTArrayFromBSON(key + "k", b, KeyFound); if (!KeyFound || msg->K.Num() != 9) return false;
		msg->R = GetDoubleTArrayFromBSON(key + "r", b, KeyFound); if (!KeyFound || msg->R.Num() != 9) return false;
		msg->P = GetDoubleTArrayFromBSON(key + "p", b, KeyFound); if (!KeyFound || msg->P.Num() != 12) return false;
		
		msg->binning_x = GetInt32FromBSON(key + "binning_x", b, KeyFound); if (!KeyFound) return false;
		msg->binning_y = GetInt32FromBSON(key + "binning_y", b, KeyFound); if (!KeyFound) return false;
		if (!USensorMsgsRegionOfInterestConverter::_bson_extract_child_roi(b, key + ".roi", &msg->roi)) return false;

		return true;
	}

	static void _bson_append_child_camera_info(bson_t *b, const char *key, const ROSMessages::sensor_msgs::CameraInfo *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_camera_info(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_camera_info(bson_t *b, const ROSMessages::sensor_msgs::CameraInfo *msg)
	{
		// assert(CastMsg->D.Num() == 5); // TODO: use Unreal assertions
		assert(CastMsg->K.Num() == 9); // TODO: use Unreal assertions
		assert(CastMsg->R.Num() == 9);
		assert(CastMsg->P.Num() == 12);
		
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_INT32(b, "height", msg->height);
		BSON_APPEND_INT32(b, "width", msg->width);
		BSON_APPEND_UTF8(b, "distortion_model", TCHAR_TO_UTF8(*msg->distortion_model));
		_bson_append_double_tarray(b, "d", msg->D);
		_bson_append_double_tarray(b, "k", msg->K);
		_bson_append_double_tarray(b, "r", msg->R);
		_bson_append_double_tarray(b, "p", msg->P);	
		BSON_APPEND_INT32(b, "binning_x", msg->binning_x);
		BSON_APPEND_INT32(b, "binning_y", msg->binning_y);
		USensorMsgsRegionOfInterestConverter::_bson_append_child_roi(b, "roi", &msg->roi);
	}
};