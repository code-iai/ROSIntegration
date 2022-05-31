#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/String.h"
#include "StdMsgsStringConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsStringConverter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsStringConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
};