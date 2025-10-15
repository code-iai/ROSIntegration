#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/Int32.h"
#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"

#include "MoveItMessageConverter.generated.h"
UCLASS()
class UMoveItMessageConverter : public UBaseMessageConverter
{
	GENERATED_BODY()
public:
	UMoveItMessageConverter();
	~UMoveItMessageConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg>& BaseMsg) override;
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) override;
};
