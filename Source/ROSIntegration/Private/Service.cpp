#include "RI/Service.h"

#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_service.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "Conversion/Services/BaseResponseConverter.h"

static TMap<FString, UBaseRequestConverter*> RequestConverterMap;
static TMap<FString, UBaseResponseConverter*> ResponseConverterMap;

// PIMPL
class UService::Impl {
	// hidden implementation details
public:
	Impl(TSharedPtr<UService, ESPMode::ThreadSafe> Service)
        : _SelfPtr(Service)
        , _HandleRequestsInGameThread(false)
        , _Ric(nullptr)
        , _ROSService(nullptr)
        , _ResponseConverter(nullptr)
        , _RequestConverter(nullptr) {
	}

    ~Impl() {
        delete _ROSService;
    }

    TWeakPtr<UService, ESPMode::ThreadSafe> _SelfPtr;
    bool _HandleRequestsInGameThread;
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

	void ServiceRequestCallback(ROSBridgeCallServiceMsg &req) {

		TSharedPtr<FROSBaseServiceRequest> ServiceRequest = _RequestConverter->AllocateConcreteRequest();

        // Convert the incoming service request to the UnrealRI format
		if (!_RequestConverter->ConvertIncomingRequest(req, ServiceRequest)) {
			UE_LOG(LogROS, Error, TEXT("Failed to Convert ROSBridgeCallServiceMsg to Unreal Service Format"));
		}

		if (!ServiceRequest.IsValid()) {
			UE_LOG(LogROS, Error, TEXT("ServiceRequest is empty after ConvertIncomingRequest - Check that AllocateConcreteRequest returns a valid instance of your Request class"));
			return;
		}

        std::string service = req.service_;
        std::string id = req.id_;
        auto CreateAndSendResponse = [this, ServiceRequest, service, id]()
        {
            if (!_SelfPtr.IsValid()) return;

            TSharedPtr<FROSBaseServiceResponse> ServiceResponse = _ResponseConverter->AllocateConcreteResponse();

            // Call the user defined Service Handler with
            _ServiceRequestCallback(ServiceRequest, ServiceResponse);

            ROSBridgeServiceResponseMsg response(true);
            response.service_ = service;
            if (id != "") response.id_ = id;
            response.values_bson_ = bson_new();

            if (!_ResponseConverter->ConvertOutgoingResponse(ServiceResponse, response)) {
                UE_LOG(LogROS, Error, TEXT("Failed to encode UnrealRI service response"));
                return;
            }

            if (!ServiceResponse.IsValid()) {
                UE_LOG(LogROS, Error, TEXT("ServiceResponse is empty after ConvertOutgoingResponse - Check that AllocateConcreteResponse returns a valid instance of your Response class"));
                return;
            }

            _Ric->_Implementation->_Ros.SendMessage(response);
        };

        if (_HandleRequestsInGameThread) {
            AsyncTask(ENamedThreads::GameThread, [this, CreateAndSendResponse]()
            {
                if (!_SelfPtr.IsValid()) return;
                CreateAndSendResponse();
            });
        }
        else
        {
            if (_SelfPtr.IsValid())
            {
                CreateAndSendResponse();
            }
        }
	}

	bool Advertise(std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> ServiceHandler, bool HandleRequestsInGameThread) {
        _ServiceRequestCallback = ServiceHandler;
        _HandleRequestsInGameThread = HandleRequestsInGameThread;
		auto service_request_handler = [this](ROSBridgeCallServiceMsg &message) { this->ServiceRequestCallback(message); };
		return _ROSService->Advertise(service_request_handler);
	}

    bool Unadvertise() {
        return _ROSService->Unadvertise();
    }

	bool CallService(
        TSharedPtr<FROSBaseServiceRequest> ServiceRequest, 
        std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {

		bson_t* service_params;

		if (!_RequestConverter->ConvertOutgoingRequest(ServiceRequest, &service_params)) {
			UE_LOG(LogROS, Error, TEXT("Failed to Convert Service call to BSON"));
            return false;
		}

        
        auto service_response_handler = [this, ServiceResponse](const ROSBridgeServiceResponseMsg &message)
        {
            if (!_SelfPtr.IsValid()) return;
            this->CallServiceCallback(message, ServiceResponse);
        };
		return _ROSService->CallService(service_params, service_response_handler);
	}
};


bool UService::CallService(TSharedPtr<FROSBaseServiceRequest> ServiceRequest, std::function<void(TSharedPtr<FROSBaseServiceResponse>)> ServiceResponse) {
    return _State.Connected && _Implementation->CallService(ServiceRequest, ServiceResponse);
}

bool UService::Advertise(std::function<void(TSharedPtr<FROSBaseServiceRequest>, TSharedPtr<FROSBaseServiceResponse>)> ServiceHandler, bool HandleRequestsInGameThread) {
    _State.Advertised = true;
    return _State.Connected && _Implementation->Advertise(ServiceHandler, HandleRequestsInGameThread);
}

bool UService::Unadvertise() {
    _State.Advertised = false;
    return _Implementation->Unadvertise();
}


UService::UService(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
    , _SelfPtr(this)
    , _Implementation(new UService::Impl(_SelfPtr))
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
    _Implementation = new UService::Impl(_SelfPtr);

    _State.Connected = true;

    _Implementation->Init(ROSIntegrationCore, oldImplementation->_ServiceName, oldImplementation->_ServiceType);
    if (_State.Advertised)
    {
        success = success && Advertise(oldImplementation->_ServiceRequestCallback, oldImplementation->_HandleRequestsInGameThread);
    }

    _State.Connected = success;

    delete oldImplementation;
    return success;
}

FString UService::GetDetailedInfoInternal() const
{
    return _Implementation->_ServiceName;
}
