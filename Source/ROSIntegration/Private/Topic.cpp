#include "RI/Topic.h"
#include "bson.h" 
#include "rosbridge2cpp/rosbridge_handler.h"



// PIMPL
class UTopic::Impl {
	// hidden implementation details
public:
	Impl()  : b(true){

	}
	ROSBridgeHandler _Handler;
	bool b;

	std::function<void(FROSBaseMsg&)> _callback;

	void ConvertMessage(FROSBaseMsg &BaseMsg, float test) {

	}

	void ConvertMessage(float test, FROSBaseMsg &BaseMsg) {

	}


	void Subscribe(std::function<void(FROSBaseMsg&)> func) {
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
	FString HandlerString(_Implementation->_Handler._TestString.c_str());
	UE_LOG(LogTemp, Warning, TEXT("Handler String is %s"), *HandlerString);
	//ROSMessages::std_msgs::String str;
	//FROSString str;

	_Implementation->_Handler.publish();

	FString HandlerString2(_Implementation->_Handler._TestString.c_str());
	UE_LOG(LogTemp, Warning, TEXT("Handler String is now %s"), *HandlerString2);
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
	//Generate BSON from ROSBaseMsg;
	//_topic.publish(BSON);
}
