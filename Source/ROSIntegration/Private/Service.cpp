#include "RI/Service.h"

#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_service.h"
#include "rospy_tutorials/AddTwoIntsRequest.h"
#include "rospy_tutorials/AddTwoIntsResponse.h"

// PIMPL
class UService::Impl {
	// hidden implementation details
public:
	Impl() : b(true) {

	}
	//ROSBridgeHandler _Handler;
	bool b;
	UROSIntegrationCore* _Ric;
	FString _ServiceName;
	FString _ServiceType;
	rosbridge2cpp::ROSService* _ROSService;
	std::function<void(TSharedPtr<FROSBaseServiceResponse>)> _LastCallServiceCallback;

	//std::function<void(TSharedPtr<FROSBaseMsg>)> _Callback;
	void Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType) {
		_Ric = Ric;
		_ServiceName = ServiceName;
		_ServiceType = ServiceType;

		_ROSService = new rosbridge2cpp::ROSService(Ric->_Implementation->_Ros, TCHAR_TO_UTF8(*ServiceName), TCHAR_TO_UTF8(*ServiceType));
	}

	void CallServiceCallback(const ROSBridgeServiceResponseMsg &message) {
		UE_LOG(LogTemp, Warning, TEXT("RECEIVED SERVICE RESPONSE"));

		bool key_found = false;

		if (_ServiceType == TEXT("rospy_tutorials/AddTwoInts")) {
			//auto ResponseMsg = MakeShareable(new FAddTwoIntsResponse());
			TSharedPtr<rospy_tutorials::FAddTwoIntsResponse> ResponseMsg = MakeShareable(new rospy_tutorials::FAddTwoIntsResponse);
			ResponseMsg->_Result = message.result_;

			ResponseMsg->_sum = rosbridge2cpp::Helper::get_int32_by_key("values.sum", *message.full_msg_bson_, key_found);
			if (!key_found) {
				UE_LOG(LogTemp, Error, TEXT("Key values.sum not present in data"));
				return;
			}
			_LastCallServiceCallback(ResponseMsg);
		}
		else {
			UE_LOG(LogTemp, Error, TEXT("ServiceType is unknown. Can't decode message"));
		}
		//ASSERT_EQ(Helper::get_int32_by_key("values.sum",*message.full_msg_bson_,key_found),42);
		//ASSERT_TRUE(key_found);

		//message.result_
		// Extract info
		// Create Concrete Request Type from it
		// Call user-given callback

		//_LastCallServiceCallback;
	}

	void CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {
		
		_LastCallServiceCallback = ServiceResponse;
		bson_t *service_params;
		if (_ServiceType == TEXT("rospy_tutorials/AddTwoInts")) {
			auto AddTwoIntsRequest = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsRequest>(ServiceRequest);
			service_params = BCON_NEW(
				"a", BCON_INT32( AddTwoIntsRequest->_a ),
				"b", BCON_INT32( AddTwoIntsRequest->_b )
			);
		}
		else {
			UE_LOG(LogTemp, Error, TEXT("ServiceType is unknown. Can't encode message in CallService"));
			return;
		}

		//CallServiceCallback
		_ROSService->CallService(service_params, std::bind(&UService::Impl::CallServiceCallback, this, std::placeholders::_1));
	}



	//typedef std::function<void(ROSBridgeCallServiceMsg&, ROSBridgeServiceResponseMsg&, rapidjson::Document::AllocatorType&)> FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator;
	//typedef std::function<void(ROSBridgeCallServiceMsg&, ROSBridgeServiceResponseMsg&)> FunVrROSCallServiceMsgrROSServiceResponseMsg;

};


void UService::doAnything() {
	UE_LOG(LogTemp, Warning, TEXT("Do Anything in Service"));
}

void UService::CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {
	_Implementation->CallService(ServiceRequest, ServiceResponse);
}

UService::UService(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_Implementation = new UService::Impl;
}

void UService::Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType) {
	_Implementation->Init(Ric, ServiceName, ServiceType);
}