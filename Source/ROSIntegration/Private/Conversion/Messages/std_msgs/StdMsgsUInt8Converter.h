#pragma once

#include <CoreMinimal.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/UInt8.h"
#include "StdMsgsUInt8Converter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsUInt8Converter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsUInt8Converter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
};
