#include "RI/Service.h"

#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_service.h"
#include "rospy_tutorials/AddTwoIntsRequest.h"
#include "rospy_tutorials/AddTwoIntsResponse.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "Conversion/Services/BaseResponseConverter.h"

static TMap<FString, UBaseRequestConverter*> RequestConverterMap;
static TMap<FString, UBaseResponseConverter*> ResponseConverterMap;

// PIMPL
class UService::Impl {
	// hidden implementation details
public:
	Impl()
        : _Ric(nullptr)
        , _ROSService(nullptr)
        , _ResponseConverter(nullptr)
        , _RequestConverter(nullptr) {
	}

    ~Impl() {
        delete _ROSService;
    }
	UROSIntegrationCore* _Ric;
	FString _ServiceName;
	FString _ServiceType;
	rosbridge2cpp::ROSService* _ROSService;
	std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> _ServiceRequestCallback;

    UBaseResponseConverter* _ResponseConverter;
    UBaseRequestConverter* _RequestConverter;

	void Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType) {
		_Ric = Ric;
		_ServiceName = ServiceName;
		_ServiceType = ServiceType;

		_ROSService = new rosbridge2cpp::ROSService(Ric->_Implementation->_Ros, TCHAR_TO_UTF8(*ServiceName), TCHAR_TO_UTF8(*ServiceType));

        // Construct static ConverterMaps
        if (RequestConverterMap.Num() == 0)
        {
            for (TObjectIterator<UClass> It; It; ++It)
            {
                UClass* ClassItr = *It;

                if (It->IsChildOf(UBaseRequestConverter::StaticClass()) && *It != UBaseRequestConverter::StaticClass())
                {
                    UBaseRequestConverter* ConcreteConverter = ClassItr->GetDefaultObject<UBaseRequestConverter>();
                    UE_LOG(LogROS, Verbose, TEXT("Added %s with type %s to RequestConverterMap"), *(It->GetDefaultObjectName().ToString()), *(ConcreteConverter->_ServiceType));
                    RequestConverterMap.Add(*(ConcreteConverter->_ServiceType), ConcreteConverter);
                    continue;
                }
                else if (It->IsChildOf(UBaseResponseConverter::StaticClass()) && *It != UBaseResponseConverter::StaticClass())
                {
                    UBaseResponseConverter* ConcreteConverter = ClassItr->GetDefaultObject<UBaseResponseConverter>();
                    UE_LOG(LogROS, Verbose, TEXT("Added %s with type %s to ResponseConverterMap"), *(It->GetDefaultObjectName().ToString()), *(ConcreteConverter->_ServiceType));
                    ResponseConverterMap.Add(*(ConcreteConverter->_ServiceType), ConcreteConverter);
                    continue;
                }
            }
        }

        UBaseResponseConverter** ResponseConverter = ResponseConverterMap.Find(_ServiceType);
        if (!ResponseConverter) {
            UE_LOG(LogROS, Error, TEXT("ServiceType is unknown. Can't find Converter to encode service call"));
            return;
        }
        _ResponseConverter = *ResponseConverter;

        UBaseRequestConverter** RequestConverter = RequestConverterMap.Find(_ServiceType);
        if (!RequestConverter) {
            UE_LOG(LogROS, Error, TEXT("ServiceType is unknown. Can't find Converter to decode service call"));
            return;
        }
        _RequestConverter = *RequestConverter;
	}

	void CallServiceCallback(const ROSBridgeServiceResponseMsg &message, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {
		TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response =
			TSharedRef<TSharedPtr<FROSBaseServiceResponse>>(new TSharedPtr<FROSBaseServiceResponse>());

		if (!_ResponseConverter->ConvertIncomingResponse(message, Response)) {
			UE_LOG(LogROS, Error, TEXT("Failed to Convert ROSBridgeCallServiceMsg to UnrealRI Service Format"));
		}

        ServiceResponse(*Response);
	}

	void ServiceRequestCallback(ROSBridgeCallServiceMsg &req, ROSBridgeServiceResponseMsg &message) {
		TSharedPtr<FROSBaseServiceRequest> ServiceRequest;
		TSharedPtr<FROSBaseServiceResponse> ServiceResponse;

		// Convert the incoming service request to the UnrealRI format

		ServiceRequest = _RequestConverter->AllocateConcreteRequest();

		if (!_RequestConverter->ConvertIncomingRequest(req, ServiceRequest)) {
			UE_LOG(LogROS, Error, TEXT("Failed to Convert ROSBridgeCallServiceMsg to Unreal Service Format"));
		}

		if (!ServiceRequest.IsValid()) {
			UE_LOG(LogROS, Error, TEXT("ServiceRequest is empty after ConvertIncomingRequest - Check that AllocateConcreteRequest returns a valid instance of your Request class"));
			return;
		}

		ServiceResponse = _ResponseConverter->AllocateConcreteResponse();

		// Call the user defined Service Handler with 
		_ServiceRequestCallback(ServiceRequest, ServiceResponse);

		if (!_ResponseConverter->ConvertOutgoingResponse(ServiceResponse, message)) {
			UE_LOG(LogROS, Error, TEXT("Failed to encode UnrealRI service response"));
			return;
		}

		if (!ServiceResponse.IsValid()) {
			UE_LOG(LogROS, Error, TEXT("ServiceResponse is empty after ConvertOutgoingResponse - Check that AllocateConcreteResponse returns a valid instance of your Response class"));
			return;
		}
	}

	bool Advertise(std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> ServiceHandler) {
        _ServiceRequestCallback = ServiceHandler;
		auto service_request_handler = [this](ROSBridgeCallServiceMsg &message, ROSBridgeServiceResponseMsg &response) { this->ServiceRequestCallback(message, response); };
		return _ROSService->Advertise(service_request_handler);
	}

    bool Unadvertise() {
        return _ROSService->Unadvertise();
    }

	bool CallService(
        TSharedPtr<FROSBaseServiceRequest> ServiceRequest, 
        std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse,
        TWeakPtr<UService, ESPMode::ThreadSafe> SelfPtr) {

		bson_t* service_params;

		if (!_RequestConverter->ConvertOutgoingRequest(ServiceRequest, &service_params)) {
			UE_LOG(LogROS, Error, TEXT("Failed to Convert Service call to BSON"));
            return false;
		}

        auto service_response_handler = [this, ServiceResponse, SelfPtr](const ROSBridgeServiceResponseMsg &message)
        {
            if (!SelfPtr.IsValid()) return;
            this->CallServiceCallback(message, ServiceResponse);
        };
		return _ROSService->CallService(service_params, service_response_handler);
	}
};


bool UService::CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {
    return _State.Connected && _Implementation->CallService(ServiceRequest, ServiceResponse, _SelfPtr);
}

bool UService::Advertise(std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> ServiceHandler) {
    _State.Advertised = true;
    return _State.Connected && _Implementation->Advertise(ServiceHandler);
}

bool UService::Unadvertise() {
    _State.Advertised = false;
    return _Implementation->Unadvertise();
}


UService::UService(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
    , _Implementation(new UService::Impl)
    , _SelfPtr(this)
{
    _State.Connected = true;
    _State.Advertised = false;
}

void UService::BeginDestroy() {
    Super::BeginDestroy();

    if (!_State.Connected)
    {
        // prevent any interaction with ROS during destruction
        _Implementation->_Ric = nullptr;
    }

    delete _Implementation;

    _SelfPtr.Reset();
}

void UService::Init(UROSIntegrationCore *Ric, FString ServiceName, FString ServiceType) {
	_Implementation->Init(Ric, ServiceName, ServiceType);
}

void UService::MarkAsDisconnected()
{
    _State.Connected = false;
}

bool UService::Reconnect(UROSIntegrationCore* ROSIntegrationCore)
{
    bool success = true;

    Impl* oldImplementation = _Implementation;
    _Implementation = new UService::Impl();

    _State.Connected = true;

    _Implementation->Init(ROSIntegrationCore, oldImplementation->_ServiceName, oldImplementation->_ServiceType);
    if (_State.Advertised)
    {
        success = success && Advertise(oldImplementation->_ServiceRequestCallback);
    }

    _State.Connected = success;

    delete oldImplementation;
    return success;
}

FString UService::GetDetailedInfoInternal() const
{
    return _Implementation->_ServiceName;
}
