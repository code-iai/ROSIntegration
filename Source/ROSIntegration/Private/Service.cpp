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
	std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> _LastServiceRequestCallback;

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

	void ServiceRequestCallback(ROSBridgeCallServiceMsg &req, ROSBridgeServiceResponseMsg &message) {
		UE_LOG(LogTemp, Warning, TEXT("Received Service request!"));
		//TSharedPtr<FROSBaseServiceRequest> ServiceRequest;/* = MakeShareable(new FROSBaseServiceRequest);*/
		TSharedPtr<FROSBaseServiceResponse> ServiceResponse = MakeShareable(new FROSBaseServiceResponse);

		// Convert rosbridge2cpp Service Request Data to Unreal Data Format
		// TODO similar to Topics (incoming message converted to Unreal Format)

		bool key_found = false;

		if (_ServiceType == TEXT("rospy_tutorials/AddTwoInts")) {
			// Get Specific Request Class
			TSharedPtr<rospy_tutorials::FAddTwoIntsRequest> ServiceRequest = MakeShareable(new rospy_tutorials::FAddTwoIntsRequest);
			TSharedPtr<rospy_tutorials::FAddTwoIntsResponse> ServiceResponse = MakeShareable(new rospy_tutorials::FAddTwoIntsResponse);

			// Retrieve attributes from wire-representation of class
			ServiceRequest->_a = rosbridge2cpp::Helper::get_int32_by_key("args.a", *(req.full_msg_bson_), key_found);
			if (!key_found) {
				UE_LOG(LogTemp, Error, TEXT("Key args.a not present in data"));
				return;
			}
			UE_LOG(LogTemp, Error, TEXT("Request.a is %d"), ServiceRequest->_a);

			ServiceRequest->_b = rosbridge2cpp::Helper::get_int32_by_key("args.b", *(req.full_msg_bson_), key_found);
			if (!key_found) {
				UE_LOG(LogTemp, Error, TEXT("Key args.b not present in data"));
				return;
			}
			UE_LOG(LogTemp, Error, TEXT("Request.b is %d"), ServiceRequest->_b);

			// Call the user defined Service Handler with 
			_LastServiceRequestCallback(ServiceRequest, ServiceResponse);

			// Convert Unreal Data Format Service Request Data to rosbridge2cpp
			message.result_ = ServiceResponse->_Result;
			BSON_APPEND_INT32(message.values_bson_, "sum", ServiceResponse->_sum);


		}
		else {
			UE_LOG(LogTemp, Error, TEXT("MessageType is unknown. Can't decode message"));
			return;
		}



	}

	void Advertise(std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> ServiceHandler) {
		_LastServiceRequestCallback = ServiceHandler;
		//typedef std::function<void(ROSBridgeCallServiceMsg&, ROSBridgeServiceResponseMsg&)> FunVrROSCallServiceMsgrROSServiceResponseMsg;
		auto service_request_handler = [this](ROSBridgeCallServiceMsg &message, ROSBridgeServiceResponseMsg &response) { this->ServiceRequestCallback(message, response); };
		//_ROSService->Advertise(std::bind(&UService::Impl::ServiceRequestCallback, this, std::placeholders::_1, std::placeholders::_2));
		_ROSService->Advertise(service_request_handler);
	}

	void CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {
		
		// Convert Unreal Data Format Service Request to rosbridge2cpp 
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
	

};


void UService::doAnything() {
	UE_LOG(LogTemp, Warning, TEXT("Do Anything in Service"));
}

void UService::CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {
	_Implementation->CallService(ServiceRequest, ServiceResponse);
}

void UService::Advertise(std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> ServiceHandler) {
	_Implementation->Advertise(ServiceHandler);
}

UService::UService(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	_Implementation = new UService::Impl;
}

void UService::Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType) {
	_Implementation->Init(Ric, ServiceName, ServiceType);
}