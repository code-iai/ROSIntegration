#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Int64.h"
#include "StdMsgsInt64Converter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsInt64Converter: public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsInt64Converter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
};
