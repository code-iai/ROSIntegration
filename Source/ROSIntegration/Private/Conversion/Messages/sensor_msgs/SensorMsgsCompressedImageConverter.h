#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>
#include "sensor_msgs/CompressedImage.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"


#include "SensorMsgsCompressedImageConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API USensorMsgsCompressedImageConverter : public UBaseMessageConverter
{
	GENERATED_UCLASS_BODY()

public:
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_image(bson_t *b, FString key, ROSMessages::sensor_msgs::CompressedImage *img)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsHeaderConverter::_bson_extract_child_header(b, TEXT("msg.header"), &img->header); if (!KeyFound) return false;

		img->format = GetFStringFromBSON(key + ".format", b, KeyFound); if (!KeyFound) return false;

		uint32_t binSize = 0;
		img->data = rosbridge2cpp::Helper::get_binary_by_key(TCHAR_TO_UTF8(*(key + ".data")), *b, binSize, KeyFound);

		return KeyFound;
	}
};
