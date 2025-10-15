#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Int32.h"
#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"
#include "MessageConverter.generated.h"

UCLASS()
class UMessageConverter : public UBaseMessageConverter 
{
	GENERATED_BODY()
public:
	UMessageConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);
};
