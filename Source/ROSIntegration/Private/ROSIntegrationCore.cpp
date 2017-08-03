#include "ROSIntegrationCore.h"


UROSIntegrationCore::UROSIntegrationCore(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

void UROSIntegrationCore::Init() {
	if (bson_test_mode) {
		 UE_LOG(LogTemp, Warning, TEXT("BSON mode enabled"));
		// OUT_INFO(TEXT("BSON mode enabled"));
		_Ros.enable_bson_mode();
	}
	bool ConnectionSuccessful = _Ros.Init("192.168.178.59", 9090);
	if (!ConnectionSuccessful) {
		UE_LOG(LogTemp, Error, TEXT("Failed to connect to server. Abort ROSBridge Init... Please make sure that your rosbridge is running."));
		return;
	} else {
		UE_LOG(LogTemp, Error, TEXT("rosbridge2cpp init successful"));
	}

	_Topic = new rosbridge2cpp::ROSTopic(_Ros, "/newtest", "std_msgs/String");
	_Topic->Subscribe(std::bind(&UROSIntegrationCore::MessageCallback, this, std::placeholders::_1));


	//test_topic_listener_.Publish(message);
	//   test_topic_listener_.Subscribe(std::bind(&ROSPluginCore::MessageCallback, this, std::placeholders::_1));


	//handler_thread_ = std::move(std::thread([=]() {HandlerThread(); return 1; }));
	//thread_set_up_ = true;
}

void UROSIntegrationCore::MessageCallback(const ROSBridgePublishMsg &message) {
	UE_LOG(LogTemp, Warning, TEXT("Message received!"));
	//std::cout << "Message received: " << std::endl;
	std::string data;
	if (bson_test_mode) {
		bool key_found;
		data = rosbridge2cpp::Helper::get_utf8_by_key("msg.data", *message.full_msg_bson_, key_found);
		if (!key_found) {
			std::cout << "Key msg.data not present in data" << std::endl;
		}
		else {
			std::cout << data << std::endl;
		}
	}/*
	else {
		data = message.msg_json_["data"].GetString();
		std::cout << data << std::endl;
	}*/
}

void UROSIntegrationCore::BeginDestroy() {
	Super::BeginDestroy();

	if(_Topic) delete _Topic;

	//if (thread_set_up_) {
	//	shutdown_thread_ = true;
	//	handler_thread_.join();
	//}
}