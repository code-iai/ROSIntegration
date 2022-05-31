#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Float32.h"
#include "StdMsgsFloat32Converter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsFloat32Converter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsFloat32Converter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
};