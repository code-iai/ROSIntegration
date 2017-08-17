#include "ROSIntegrationGameInstance.h"
#include "std_msgs/String.h"
#include "bson.h" 


void UROSIntegrationGameInstance::Init() {
	UE_LOG(LogTemp, Warning, TEXT("Init on GameInstance"));
	//ROSMessages::std_msgs::String str;
	//FROSString str;
	 
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
	UE_LOG(LogTemp, Warning, TEXT("BSON Output is %s"), *str_in_unreal);

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

	_Ric = NewObject<UROSIntegrationCore>(UROSIntegrationCore::StaticClass());
	_Ric->Init();

}

void UROSIntegrationGameInstance::Shutdown() {
	UE_LOG(LogTemp, Warning, TEXT("Shutdown on GameInstance"));
	//if (_Ric != nullptr)
	//	delete _Ric;
}

void UROSIntegrationGameInstance::BeginDestroy() {
	UE_LOG(LogTemp, Warning, TEXT("BeginDestroy on GameInstance"));
	Super::BeginDestroy();
}