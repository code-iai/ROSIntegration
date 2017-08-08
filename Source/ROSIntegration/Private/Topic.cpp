#include "RI/Topic.h"
#include "bson.h" 
//#include "rosbridge2cpp/rosbridge_handler.h"
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"



// PIMPL
class UTopic::Impl {
	// hidden implementation details
public:
	Impl()  : b(true){

	}
	//ROSBridgeHandler _Handler;
	bool b;
	UROSIntegrationCore* _Ric;
	FString _Topic;
	FString _MessageType;
	rosbridge2cpp::ROSTopic* _ROSTopic;

	std::function<void(FROSBaseMsg&)> _Callback;

	void ConvertMessage(FROSBaseMsg &BaseMsg, float test) {

	}

	void ConvertMessage(float test, FROSBaseMsg &BaseMsg) {

	}


	void Subscribe(std::function<void(FROSBaseMsg&)> func) {
		if (!_ROSTopic) {
			UE_LOG(LogTemp, Error, TEXT("Rostopic hasn't been initialized before Subscribe() call"));
			return;
		}

		_ROSTopic->Subscribe(std::bind(&UTopic::Impl::MessageCallback, this, std::placeholders::_1));
		_Callback = func;
		/*_topic->Subscribe(std::bind(&FROSTopic:::ConvertMessageCallback, this, std::placeholders::_1));
		_callback = fun(ROSBaseMsg);*/
	}
	void Unsubscribe(std::function<void(FROSBaseMsg&)> func) {

	}

	void Advertise() {
		//Advertise() { _topic.advertise(); }
	}


	void Unadvertise() {
		//Unadvertise() { _topic.unadvertise(); }
	}


	void Publish(FROSBaseMsg& msg) {
		//Generate BSON from ROSBaseMsg;
		//_topic.publish(BSON);
	}

	void Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType) {
		_Ric = Ric;
		_Topic = Topic;
		_MessageType = MessageType;

		_ROSTopic = new rosbridge2cpp::ROSTopic(Ric->_Implementation->_Ros, "/newtest", "std_msgs/String");
		
	}

	void MessageCallback(const ROSBridgePublishMsg &message) {
		UE_LOG(LogTemp, Warning, TEXT("Topic Message received!"));

		// TODO Use factory
		if (_MessageType == TEXT("std_msgs/String")) {
			
			bool key_found;
			std::string data = rosbridge2cpp::Helper::get_utf8_by_key("msg.data", *message.full_msg_bson_, key_found);
			if (!key_found) {
				std::cout << "Key msg.data not present in data" << std::endl;
			}else {
				ROSMessages::std_msgs::String StringMessage(UTF8_TO_TCHAR(data.c_str()));
				_Callback(StringMessage);
			}
		}
		else {
			UE_LOG(LogTemp, Error, TEXT("MessageType is unknown. Can't decode message"));
		}

		UE_LOG(LogTemp, Warning, TEXT("Callback done"));
	}
};

// Interface Implementation

UTopic::UTopic(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_Implementation = new UTopic::Impl;
}

void UTopic::BeginDestroy() {
	Super::BeginDestroy();

	delete _Implementation;
}

void UTopic::doSomething() {
	UE_LOG(LogTemp, Warning, TEXT("doSomething"));
	//FString HandlerString(_Implementation->_Handler._TestString.c_str());
	//UE_LOG(LogTemp, Warning, TEXT("Handler String is %s"), *HandlerString);
	//ROSMessages::std_msgs::String str;
	//FROSString str;

	//_Implementation->_Handler.publish();

	//FString HandlerString2(_Implementation->_Handler._TestString.c_str());
	//UE_LOG(LogTemp, Warning, TEXT("Handler String is now %s"), *HandlerString2);
	bson_t parent;
	bson_t child;
	char *str;
	bson_init(&parent);
	bson_append_document_begin(&parent, "foo", 3, &child);
	bson_append_int32(&child, "baz", 3, 1);
	bson_append_document_end(&parent, &child);

	str = bson_as_json(&parent, NULL);
	//printf("%s\n", str);
	FString str_in_unreal(str);
	UE_LOG(LogTemp, Warning, TEXT("something BSON Output is %s"), *str_in_unreal);

	bson_iter_t iter;
	bson_iter_t baz;

	if (bson_iter_init(&iter, &parent) &&
		bson_iter_find_descendant(&iter, "foo.baz", &baz) &&
		BSON_ITER_HOLDS_INT32(&baz)) {
		UE_LOG(LogTemp, Warning, TEXT("foo baz is %d"), bson_iter_int32(&baz));
		//printf("baz = %d\n", bson_iter_int32(&baz));
	}
	bson_free(str);

	bson_destroy(&parent);

	
}



void UTopic::Subscribe(std::function<void(FROSBaseMsg&)> func) {
	/*_topic->Subscribe(std::bind(&FROSTopic:::ConvertMessageCallback, this, std::placeholders::_1));
	_callback = fun(ROSBaseMsg);*/
	_Implementation->Subscribe(func);
}
void UTopic::Unsubscribe(std::function<void(FROSBaseMsg&)> func) {

}

void UTopic::Advertise() {
	//Advertise() { _topic.advertise(); }
}
void UTopic::Unadvertise() {
	//Unadvertise() { _topic.unadvertise(); }
}
void UTopic::Publish(FROSBaseMsg& msg) {
	_Implementation->Publish(msg);

}

void UTopic::Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType) {
	_Implementation->Init(Ric, Topic, MessageType);
}
