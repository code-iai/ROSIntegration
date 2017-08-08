#include "ROSIntegrationCore.h"
#include "rosbridge2cpp/TCPConnection.h"
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"
//#include "rosbridge2cpp/messages/rosbridge_publish_msg.h"

//class UROSIntegrationCore::ROSBridgeWrapper {
//public:
//	ROSBridgeWrapper() {
//
//	};
//};

// PIMPL
class UROSIntegrationCore::Impl {
	// hidden implementation details
public:
	bool _bson_test_mode;

	TCPConnection _Connection;
	rosbridge2cpp::ROSTopic* _Topic;
	rosbridge2cpp::ROSBridge _Ros{ _Connection };


	Impl() {

	}
	void Init(bool bson_test_mode) {
		_bson_test_mode = bson_test_mode;

		if (bson_test_mode) {
			UE_LOG(LogTemp, Warning, TEXT("BSON mode enabled"));
			// OUT_INFO(TEXT("BSON mode enabled"));
			_Ros.enable_bson_mode();
		}
		bool ConnectionSuccessful = _Ros.Init("192.168.178.59", 9090);
		if (!ConnectionSuccessful) {
			UE_LOG(LogTemp, Error, TEXT("Failed to connect to server. Abort ROSBridge Init... Please make sure that your rosbridge is running."));
			return;
		}
		else {
			UE_LOG(LogTemp, Error, TEXT("rosbridge2cpp init successful"));
		}

		/*_Topic = new rosbridge2cpp::ROSTopic(_Ros, "/newtest", "std_msgs/String");
		_Topic->Subscribe(std::bind(&UROSIntegrationCore::Impl::MessageCallback, this, std::placeholders::_1));*/
	}

	//void MessageCallback(const ROSBridgePublishMsg &message) {
	//	UE_LOG(LogTemp, Warning, TEXT("Message received!"));
	//	//std::cout << "Message received: " << std::endl;
	//	std::string data;
	//	if (_bson_test_mode) {
	//		bool key_found;
	//		data = rosbridge2cpp::Helper::get_utf8_by_key("msg.data", *message.full_msg_bson_, key_found);
	//		if (!key_found) {
	//			std::cout << "Key msg.data not present in data" << std::endl;
	//		}
	//		else {
	//			std::cout << data << std::endl;
	//		}
	//	}/*
	//	 else {
	//	 data = message.msg_json_["data"].GetString();
	//	 std::cout << data << std::endl;
	//	 }*/
	//	UE_LOG(LogTemp, Warning, TEXT("Callback done"));
	//}
};


UROSIntegrationCore::UROSIntegrationCore(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

void UROSIntegrationCore::Init() {
	_Implementation = new UROSIntegrationCore::Impl;
	_Implementation->Init(bson_test_mode);
	//_ROSBridgeWrapper = new UROSIntegrationCore::ROSBridgeWrapper;
}

//void UROSIntegrationCore::MessageCallback(const ROSBridgePublishMsg &message) {
//	UE_LOG(LogTemp, Warning, TEXT("PUBLIC Message received!"));
//}

void UROSIntegrationCore::BeginDestroy() {
	UE_LOG(LogTemp, Warning, TEXT("Begin Destroy on UROSIntegrationCore called"));
	Super::BeginDestroy();

	if (_Implementation) delete _Implementation;
}

//rosbridge2cpp::ROSBridge* UROSIntegrationCore::GetROSBridge() {
//	return nullptr;
//}