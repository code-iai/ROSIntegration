#include "RI/Topic.h"
#include "bson.h" 
//#include "rosbridge2cpp/rosbridge_handler.h"
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"
//#include "bson.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"


// PIMPL
class UTopic::Impl {
	// hidden implementation details
public:
	Impl() : b(true) {

	}
	//ROSBridgeHandler _Handler;
	bool b;
	UROSIntegrationCore* _Ric;
	FString _Topic;
	FString _MessageType;
	rosbridge2cpp::ROSTopic* _ROSTopic;
	TMap<FString, UBaseMessageConverter*> _ConverterMap;

	std::function<void(TSharedPtr<FROSBaseMsg>)> _Callback;

	bool ConvertMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
		// TODO do this on advertise/call?
		UBaseMessageConverter** Converter = _ConverterMap.Find(_MessageType);
		if (!Converter) {
			UE_LOG(LogTemp, Error, TEXT("MessageType is unknown. Can't find Converter to encode message"));
			return false;
		}

		return (*Converter)->ConvertOutgoingMessage(BaseMsg, message);
	}

	// IN Parameter: message
	// OUT Parameter: BaseMsg
	bool ConvertMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
		// TODO do this on advertise/call?
		UBaseMessageConverter** Converter = _ConverterMap.Find(_MessageType);
		if (!Converter) {
			UE_LOG(LogTemp, Error, TEXT("MessageType is unknown. Can't find Converter to decode message"));
			return false;
		}

		return (*Converter)->ConvertIncomingMessage(message, BaseMsg);
	}


	void Subscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func) {
		if (!_ROSTopic) {
			UE_LOG(LogTemp, Error, TEXT("Rostopic hasn't been initialized before Subscribe() call"));
			return;
		}

		_ROSTopic->Subscribe(std::bind(&UTopic::Impl::MessageCallback, this, std::placeholders::_1));
		_Callback = func;
	}
	void Unsubscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func) {

	}

	void Advertise() {
		assert(_ROSTopic);
		_ROSTopic->Advertise();
	}


	void Unadvertise() {
		assert(_ROSTopic);
		_ROSTopic->Unadvertise();
	}


	void Publish(TSharedPtr<FROSBaseMsg> msg) {
		bson_t *bson_message = nullptr;

		if (ConvertMessage(msg, &bson_message)) {
			//UE_LOG(LogTemp, Error, TEXT("Publishing Converted Message"));
			_ROSTopic->Publish(bson_message);
			//bson_destroy(bson_message); // Not necessary, since bson memory will be freed in the rosbridge core code
		}
		else {
			UE_LOG(LogTemp, Error, TEXT("Failed to ConvertMessage in UTopic::Publish()"));
		}
	}

	void Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType) {
		_Ric = Ric;
		_Topic = Topic;
		_MessageType = MessageType;

		_ROSTopic = new rosbridge2cpp::ROSTopic(Ric->_Implementation->_Ros, TCHAR_TO_UTF8(*Topic), TCHAR_TO_UTF8(*MessageType));

		// Construct ConverterMap
		for (TObjectIterator<UClass> It; It; ++It)
		{
			UClass* ClassItr = *It;

			if (It->IsChildOf(UBaseMessageConverter::StaticClass()) && *It != UBaseMessageConverter::StaticClass())
			{
				UBaseMessageConverter* ConcreteConverter = ClassItr->GetDefaultObject<UBaseMessageConverter>();
				UE_LOG(LogTemp, Warning, TEXT("Added %s with type %s to TopicConverterMap"), *(It->GetDefaultObjectName().ToString()), *(ConcreteConverter->_MessageType));
				_ConverterMap.Add(*(ConcreteConverter->_MessageType), ConcreteConverter);


			}
		}

	}

	void MessageCallback(const ROSBridgePublishMsg &message) {
		UE_LOG(LogTemp, Warning, TEXT("Topic Message received!"));

		TSharedPtr<FROSBaseMsg> BaseMsg;
		if (ConvertMessage(&message, BaseMsg)) {
			UE_LOG(LogTemp, Warning, TEXT("AFTER CONVERT: basemsg is  %s"), *(BaseMsg->_MessageType));
			_Callback(BaseMsg);
		}
		else {
			UE_LOG(LogTemp, Error, TEXT("Couldn't convert incoming Message; Skipping callback"));
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
	////FString HandlerString(_Implementation->_Handler._TestString.c_str());
	////UE_LOG(LogTemp, Warning, TEXT("Handler String is %s"), *HandlerString);
	////ROSMessages::std_msgs::String str;
	////FROSString str;

	////_Implementation->_Handler.publish();

	////FString HandlerString2(_Implementation->_Handler._TestString.c_str());
	////UE_LOG(LogTemp, Warning, TEXT("Handler String is now %s"), *HandlerString2);
	//bson_t parent;
	//bson_t child;
	//char *str;
	//bson_init(&parent);
	//bson_append_document_begin(&parent, "foo", 3, &child);
	//bson_append_int32(&child, "baz", 3, 1);
	//bson_append_document_end(&parent, &child);

	//str = bson_as_json(&parent, NULL);
	////printf("%s\n", str);
	//FString str_in_unreal(str);
	//UE_LOG(LogTemp, Warning, TEXT("something BSON Output is %s"), *str_in_unreal);

	//bson_iter_t iter;
	//bson_iter_t baz;

	//if (bson_iter_init(&iter, &parent) &&
	//	bson_iter_find_descendant(&iter, "foo.baz", &baz) &&
	//	BSON_ITER_HOLDS_INT32(&baz)) {
	//	UE_LOG(LogTemp, Warning, TEXT("foo baz is %d"), bson_iter_int32(&baz));
	//	//printf("baz = %d\n", bson_iter_int32(&baz));
	//}
	//bson_free(str);

	//bson_destroy(&parent);


}



void UTopic::Subscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func) {
	/*_topic->Subscribe(std::bind(&FROSTopic:::ConvertMessageCallback, this, std::placeholders::_1));
	_callback = fun(ROSBaseMsg);*/
	_Implementation->Subscribe(func);
}
void UTopic::Unsubscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func) {

}

void UTopic::Advertise() {
	//Advertise() { _topic.advertise(); }
}
void UTopic::Unadvertise() {
	//Unadvertise() { _topic.unadvertise(); }
}
void UTopic::Publish(TSharedPtr<FROSBaseMsg> msg) {
	_Implementation->Publish(msg);

}

void UTopic::Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType) {
	_Implementation->Init(Ric, Topic, MessageType);
}
