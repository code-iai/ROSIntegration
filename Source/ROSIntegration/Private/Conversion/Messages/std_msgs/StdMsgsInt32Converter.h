#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Int32.h"
#include "StdMsgsInt32Converter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsInt32Converter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsInt32Converter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
};
