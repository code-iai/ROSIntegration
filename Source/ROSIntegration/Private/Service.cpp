#include "RI/Service.h"

#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_service.h"
#include "rospy_tutorials/AddTwoIntsRequest.h"
#include "rospy_tutorials/AddTwoIntsResponse.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "Conversion/Services/BaseResponseConverter.h"

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
	TMap<FString, UBaseRequestConverter*> _RequestConverterMap;
	TMap<FString, UBaseResponseConverter*> _ResponseConverterMap;

	void Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType) {
		_Ric = Ric;
		_ServiceName = ServiceName;
		_ServiceType = ServiceType;

		_ROSService = new rosbridge2cpp::ROSService(Ric->_Implementation->_Ros, TCHAR_TO_UTF8(*ServiceName), TCHAR_TO_UTF8(*ServiceType));


		// Construct Converter Maps
		for (TObjectIterator<UClass> It; It; ++It)
		{
			UClass* ClassItr = *It;

			if (It->IsChildOf(UBaseRequestConverter::StaticClass()) && *It != UBaseRequestConverter::StaticClass())
			{
				UBaseRequestConverter* ConcreteConverter = ClassItr->GetDefaultObject<UBaseRequestConverter>();
				UE_LOG(LogTemp, Verbose, TEXT("Added %s with type %s to RequestConverterMap"), *(It->GetDefaultObjectName().ToString()), *(ConcreteConverter->_ServiceType));
				_RequestConverterMap.Add(*(ConcreteConverter->_ServiceType), ConcreteConverter);
				continue;
			}
			else if (It->IsChildOf(UBaseResponseConverter::StaticClass()) && *It != UBaseResponseConverter::StaticClass())
			{
				UBaseResponseConverter* ConcreteConverter = ClassItr->GetDefaultObject<UBaseResponseConverter>();
				UE_LOG(LogTemp, Verbose, TEXT("Added %s with type %s to ResponseConverterMap"), *(It->GetDefaultObjectName().ToString()), *(ConcreteConverter->_ServiceType));
				_ResponseConverterMap.Add(*(ConcreteConverter->_ServiceType), ConcreteConverter);
				continue;
			}
		}
	}

	void CallServiceCallback(const ROSBridgeServiceResponseMsg &message) {
		UE_LOG(LogTemp, Verbose, TEXT("RECEIVED SERVICE RESPONSE"));

		TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response =
			TSharedRef<TSharedPtr<FROSBaseServiceResponse>>(new TSharedPtr<FROSBaseServiceResponse>());

		UBaseResponseConverter** Converter = _ResponseConverterMap.Find(_ServiceType);
		if (!Converter) {
			UE_LOG(LogTemp, Error, TEXT("ServiceType is unknown. Can't find Converter to encode service call"));
			return;
		}

		if (!(*Converter)->ConvertIncomingResponse(message, Response)) {
			UE_LOG(LogTemp, Error, TEXT("Failed to Convert ROSBridgeCallServiceMsg to UnrealRI Service Format"));
		}

		_LastCallServiceCallback(*Response);

	}

	void ServiceRequestCallback(ROSBridgeCallServiceMsg &req, ROSBridgeServiceResponseMsg &message) {
		TSharedPtr<FROSBaseServiceRequest> ServiceRequest;
		TSharedPtr<FROSBaseServiceResponse> ServiceResponse;

		// Convert the incoming service request to the UnrealRI format

		UBaseRequestConverter** RequestConverter = _RequestConverterMap.Find(_ServiceType);
		if (!RequestConverter) {
			UE_LOG(LogTemp, Error, TEXT("ServiceType is unknown. Can't find Converter to decode service call"));
			return;
		}

		ServiceRequest = (*RequestConverter)->AllocateConcreteRequest();

		if (!(*RequestConverter)->ConvertIncomingRequest(req, ServiceRequest)) {
			UE_LOG(LogTemp, Error, TEXT("Failed to Convert ROSBridgeCallServiceMsg to Unreal Service Format"));
		}

		if (!ServiceRequest.IsValid()) {
			UE_LOG(LogTemp, Error, TEXT("ServiceRequest is empty after ConvertIncomingRequest - Check that AllocateConcreteRequest returns a valid instance of your Request class"));
			return;
		}

		UBaseResponseConverter** ResponseConverter = _ResponseConverterMap.Find(_ServiceType);
		if (!ResponseConverter) {
			UE_LOG(LogTemp, Error, TEXT("ServiceType is unknown. Can't find Converter to encode service response"));
			return;
		}
		ServiceResponse = (*ResponseConverter)->AllocateConcreteResponse();

		// Call the user defined Service Handler with 
		_LastServiceRequestCallback(ServiceRequest, ServiceResponse);


		if (!(*ResponseConverter)->ConvertOutgoingResponse(ServiceResponse, message)) {
			UE_LOG(LogTemp, Error, TEXT("Failed to encode UnrealRI service response"));
			return;
		}

		if (!ServiceResponse.IsValid()) {
			UE_LOG(LogTemp, Error, TEXT("ServiceResponse is empty after ConvertOutgoingResponse - Check that AllocateConcreteResponse returns a valid instance of your Response class"));
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

		UBaseRequestConverter** Converter = _RequestConverterMap.Find(_ServiceType);
		if (!Converter) {
			UE_LOG(LogTemp, Error, TEXT("ServiceType is unknown. Can't find Converter to encode service call"));
			return;
		}

		_LastCallServiceCallback = ServiceResponse;
		bson_t *service_params;

		if (!(*Converter)->ConvertOutgoingRequest(ServiceRequest, &service_params)) {
			UE_LOG(LogTemp, Error, TEXT("Failed to Convert Service call to BSON"));
		}

		//CallServiceCallback
		_ROSService->CallService(service_params, std::bind(&UService::Impl::CallServiceCallback, this, std::placeholders::_1));
	}

};


void UService::doAnything() {
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