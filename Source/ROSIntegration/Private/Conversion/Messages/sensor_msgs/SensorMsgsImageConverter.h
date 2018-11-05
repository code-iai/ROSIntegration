#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "sensor_msgs/Image.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"


#include "SensorMsgsImageConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsImageConverter: public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_image(bson_t *b, FString key, ROSMessages::sensor_msgs::Image *img)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, TEXT("msg.header"), &img->header); if (!KeyFound) return false;

		img->height = GetInt32FromBSON(key + ".height", b, KeyFound); if (!KeyFound) return false;
		img->width = GetInt32FromBSON(key + ".width", b, KeyFound); if (!KeyFound) return false;
		img->encoding = GetFStringFromBSON(key + ".encoding", b, KeyFound); if (!KeyFound) return false;
		img->is_bigendian = GetInt32FromBSON(key + ".is_bigendian", b, KeyFound); if (!KeyFound) return false;
		img->step = GetInt32FromBSON(key + ".step", b, KeyFound); if (!KeyFound) return false;

		uint32_t binSize = 0;
		img->data = rosbridge2cpp::Helper::get_binary_by_key(TCHAR_TO_UTF8(*(key + ".data")), *b, binSize, KeyFound);

		return KeyFound;
	}
};
