#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "rosbridge2cpp/TCPConnection.h"
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"
#include "rosbridge2cpp/messages/rosbridge_publish_msg.h"

#include "ROSIntegrationCore.generated.h"


UCLASS()
class ROSINTEGRATION_API UROSIntegrationCore: public UObject
{
	GENERATED_UCLASS_BODY()

public:
	void Init();
	//void HandlerThread();
	void BeginDestroy() override;
	void MessageCallback(const ROSBridgePublishMsg &message);
private:

	UPROPERTY()
	bool test;


	
	bool bson_test_mode = true;
	//std::thread handler_thread_;
	//bool shutdown_thread_ = false;
	//bool thread_set_up_ = false;

	TCPConnection _Connection;
	rosbridge2cpp::ROSTopic* _Topic;

public:
	rosbridge2cpp::ROSBridge _Ros{ _Connection };

};

