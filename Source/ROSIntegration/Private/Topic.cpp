#include "RI/Topic.h"
#include <bson.h>
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsBoolConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"




static TMap<FString, UBaseMessageConverter*> TypeConverterMap;
static TMap<EMessageType, FString> SupportedMessageTypes;


// PIMPL
class UTopic::Impl {
	// hidden implementation details
public:
	Impl()
	: _Ric(nullptr)
	, _ROSTopic(nullptr)
	, _Converter(nullptr)
	{
	}

	~Impl() {

		if (_Callback && _Ric) {
			Unsubscribe();
		}

		if(_ROSTopic) delete _ROSTopic;
	}

	UROSIntegrationCore* _Ric = nullptr;
	FString _Topic;
	FString _MessageType;
	int32 _QueueSize;
	rosbridge2cpp::ROSTopic* _ROSTopic = nullptr;
	UBaseMessageConverter* _Converter;
	rosbridge2cpp::ROSCallbackHandle<rosbridge2cpp::FunVrROSPublishMsg> _CallbackHandle;

	std::function<void(TSharedPtr<FROSBaseMsg>)> _Callback;

	bool ConvertMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
	{
		return _Converter->ConvertOutgoingMessage(BaseMsg, message);
	}

	bool ConvertMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
	{
		return _Converter->ConvertIncomingMessage(message, BaseMsg);
	}

	bool Subscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func)
	{
		if (!_ROSTopic) {
			UE_LOG(LogROS, Error, TEXT("Rostopic hasn't been initialized before Subscribe() call"));
			return false;
		}
		if (_Callback) {
			UE_LOG(LogROS, Warning, TEXT("Rostopic was already subscribed"));
			Unsubscribe();
		}

		_CallbackHandle = _ROSTopic->Subscribe(std::bind(&UTopic::Impl::MessageCallback, this, std::placeholders::_1));
		_Callback = func;
		return _CallbackHandle.IsValid();
	}

	bool Unsubscribe()
	{
		if (!_ROSTopic) {
			UE_LOG(LogROS, Error, TEXT("Rostopic hasn't been initialized before Unsubscribe() call"));
			return false;
		}

		bool result = _ROSTopic->Unsubscribe(_CallbackHandle);
		if (result) {
			_Callback = nullptr;
			_CallbackHandle = rosbridge2cpp::ROSCallbackHandle<rosbridge2cpp::FunVrROSPublishMsg>();
			if(_ROSTopic) delete _ROSTopic;
			_ROSTopic = nullptr;
		}
		return result;
	}

	bool Advertise()
	{
		if (!_ROSTopic) UE_LOG(LogROS, Warning, TEXT("Trying to advertise on an un-initialized topic."))
		return _ROSTopic && _ROSTopic->Advertise();
	}


	bool Unadvertise()
	{
		if (!_ROSTopic) UE_LOG(LogROS, Warning, TEXT("Trying to unadvertise on an un-initialized topic."))
		return _ROSTopic && _ROSTopic->Unadvertise();
	}


	bool Publish(TSharedPtr<FROSBaseMsg> msg)
	{
		bson_t *bson_message = nullptr;

		if (ConvertMessage(msg, &bson_message)) {
			return _ROSTopic->Publish(bson_message); // bson memory will be freed in the rosbridge core code after the message is published
		}
		else {
			UE_LOG(LogROS, Error, TEXT("Failed to ConvertMessage in UTopic::Publish()"));
			return false;
		}
	}

	void Init(UROSIntegrationCore *Ric, const FString& Topic, const FString& MessageType, int32 QueueSize)
	{
		// Construct static ConverterMap
		if (TypeConverterMap.Num() == 0)
		{
			for (TObjectIterator<UClass> It; It; ++It)
			{
				UClass* ClassItr = *It;

				if (It->IsChildOf(UBaseMessageConverter::StaticClass()) && *It != UBaseMessageConverter::StaticClass())
				{
					UBaseMessageConverter* ConcreteConverter = ClassItr->GetDefaultObject<UBaseMessageConverter>();
					// UE_LOG(LogROS, Log, TEXT("Added %s with type %s to TopicConverterMap"), *(It->GetDefaultObjectName().ToString()), *(ConcreteConverter->_MessageType));
					TypeConverterMap.Add(*(ConcreteConverter->_MessageType), ConcreteConverter);
				}
			}
		}

		_Ric = Ric;
		_Topic = Topic;
		_MessageType = MessageType;
		_QueueSize = QueueSize;

		UBaseMessageConverter** Converter = TypeConverterMap.Find(MessageType);
		if (!Converter)
		{
			UE_LOG(LogROS,
			       Error, 
			       TEXT("MessageType [%s] for Topic [%s] "
				    "is unknown. Message ignored."), 
			       *MessageType, 
			       *Topic);
			return;
		}
		_Converter = *Converter;

		_ROSTopic = new rosbridge2cpp::ROSTopic(Ric->_Implementation->Get()->_Ros, TCHAR_TO_UTF8(*Topic), TCHAR_TO_UTF8(*MessageType), QueueSize);
	}

	void MessageCallback(const ROSBridgePublishMsg &message)
	{
		TSharedPtr<FROSBaseMsg> BaseMsg;
		if (ConvertMessage(&message, BaseMsg)) {
			_Callback(BaseMsg);
		}
		else {
			UE_LOG(LogROS, Error, TEXT("Couldn't convert incoming Message; Skipping callback"));
		}
	}
};

// Interface Implementation

UTopic::UTopic(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
, _SelfPtr(this, TDeleterNot())
, _Implementation(new UTopic::Impl())
{
	_State.Connected = true;
	_State.Advertised = false;
	_State.Subscribed = false;
	_State.Blueprint = false;

	if (SupportedMessageTypes.Num() == 0)
	{
		SupportedMessageTypes.Add(EMessageType::String,      TEXT("std_msgs/String"));
		SupportedMessageTypes.Add(EMessageType::Float32,     TEXT("std_msgs/Float32"));
		SupportedMessageTypes.Add(EMessageType::Bool,        TEXT("std_msgs/Bool"));
		SupportedMessageTypes.Add(EMessageType::Int32,       TEXT("std_msgs/Int32"));
		SupportedMessageTypes.Add(EMessageType::Int64,       TEXT("std_msgs/Int64"));

		SupportedMessageTypes.Add(EMessageType::Vector3, TEXT("geometry_msgs/Vector3"));
		SupportedMessageTypes.Add(EMessageType::Point, TEXT("geometry_msgs/Point"));
		SupportedMessageTypes.Add(EMessageType::Pose, TEXT("geometry_msgs/Pose"));
		SupportedMessageTypes.Add(EMessageType::Quaternion, TEXT("geometry_msgs/Quaternion"));
		SupportedMessageTypes.Add(EMessageType::Twist, TEXT("geometry_msgs/Twist"));

	}
}

void UTopic::PostInitProperties()
{
	Super::PostInitProperties();

	if (GetOuter()->GetWorld())
	{
		OnConstruct();
	}
}

void UTopic::BeginDestroy() {

	if (_Implementation && (!_State.Connected || !_ROSIntegrationCore || _ROSIntegrationCore->HasAnyFlags(EObjectFlags::RF_BeginDestroyed)))
	{
		// prevent any interaction with ROS during destruction
		_Implementation->_Ric = nullptr;
	}
	_State.Connected = false;

	if(_Implementation) delete _Implementation;
	_Implementation = nullptr;

	Super::BeginDestroy();

	_SelfPtr.Reset();
}

bool UTopic::Subscribe(std::function<void(TSharedPtr<FROSBaseMsg>)> func)
{
	_State.Subscribed = true;
	return _State.Connected && _Implementation->Subscribe(func);
}

bool UTopic::Unsubscribe()
{
	_State.Subscribed = false;
	return _State.Connected && _Implementation && _Implementation->Unsubscribe();
}

bool UTopic::Advertise()
{
	_State.Advertised = true;
	return _State.Connected && _Implementation->Advertise();
}

bool UTopic::Unadvertise()
{
	_State.Advertised = false;
	return _State.Connected && _Implementation->Unadvertise();
}

bool UTopic::Publish(TSharedPtr<FROSBaseMsg> msg)
{
	return _State.Connected && _Implementation->Publish(msg);
}

void UTopic::Init(UROSIntegrationCore *Ric, FString Topic, FString MessageType, int32 QueueSize)
{
	_ROSIntegrationCore = Ric;
	_Implementation->Init(Ric, Topic, MessageType, QueueSize);
}

void UTopic::MarkAsDisconnected()
{
	_State.Connected = false;
}

bool UTopic::Reconnect(UROSIntegrationCore* ROSIntegrationCore)
{
	bool success = true;
	_ROSIntegrationCore = ROSIntegrationCore;

	Impl* oldImplementation = _Implementation;
	_Implementation = new UTopic::Impl();
	_Implementation->Init(ROSIntegrationCore, oldImplementation->_Topic, oldImplementation->_MessageType, oldImplementation->_QueueSize);

	_State.Connected = true;
	if (_State.Subscribed)
	{
		success = Subscribe(oldImplementation->_Callback);
	}
	if (_State.Advertised)
	{
		success = success && Advertise();
	}
	_State.Connected = success;

	if (oldImplementation)
	{
		oldImplementation->_Ric = nullptr; // prevent old topic from unsubscribing using the broken connection
		delete oldImplementation;
	}
	return success;
}

bool UTopic::IsAdvertising() 
{
	return _State.Advertised;
}

FString UTopic::GetDetailedInfoInternal() const
{
	return _Implementation->_Topic;
}

void UTopic::Init(const FString& TopicName, EMessageType MessageType, int32 QueueSize)
{
	_State.Blueprint = true;
	_State.BlueprintMessageType = MessageType;

	UROSIntegrationGameInstance* ROSInstance = Cast<UROSIntegrationGameInstance>(GWorld->GetGameInstance());
	if (ROSInstance)
	{
		if (ROSInstance->bConnectToROS && _State.Connected)
		{
			Init(ROSInstance->ROSIntegrationCore, TopicName, SupportedMessageTypes[MessageType], QueueSize);
		}
	}
	else
	{
		UE_LOG(LogROS, Warning, TEXT("ROSIntegrationGameInstance does not exist."));
	}
}

bool UTopic::Subscribe()
{
	bool success = false;
	_State.Subscribed = true;

	if (_State.Connected)
	{
		EMessageType MessageType = _State.BlueprintMessageType;
		std::function<void(TSharedPtr<FROSBaseMsg>)> Callback = [this, MessageType](TSharedPtr<FROSBaseMsg> msg) -> void
		{
			switch (MessageType)
			{
			case EMessageType::String:
			{
				auto ConcreteStringMessage = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
				if (ConcreteStringMessage.IsValid())
				{
					const FString Data = ConcreteStringMessage->_Data;
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Data, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnStringMessage(Data);
					});
				}
				break;
			}
			case EMessageType::Float32:
			{
				auto ConcreteFloatMessage = StaticCastSharedPtr<ROSMessages::std_msgs::Float32>(msg);
				if (ConcreteFloatMessage.IsValid())
				{
					const float Data = ConcreteFloatMessage->_Data;
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Data, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnFloat32Message(Data);
					});
				}
				break;
			}
			case EMessageType::Bool:
			{
				auto ConcreteBoolMessage = StaticCastSharedPtr<ROSMessages::std_msgs::Bool>(msg);
				if (ConcreteBoolMessage.IsValid())
				{
					const bool Data = ConcreteBoolMessage->_Data;
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Data, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnBoolMessage(Data);
					});
				}
				break;
			}
			case EMessageType::Int32:
			{
				auto ConcreteInt32Message = StaticCastSharedPtr<ROSMessages::std_msgs::Int32>(msg);
				if (ConcreteInt32Message.IsValid())
				{
					const int32 Data = ConcreteInt32Message->_Data;
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Data, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnInt32Message(Data);
					});
				}
				break;
			}
			case EMessageType::Int64:
			{
				auto ConcreteInt64Message = StaticCastSharedPtr<ROSMessages::std_msgs::Int64>(msg);
				if (ConcreteInt64Message.IsValid())
				{
					const int64 Data = ConcreteInt64Message->_Data;
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Data, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnInt64Message(Data);
					});
				}
				break;
			}
			case EMessageType::Vector3:
			{
				auto ConcreteVectorMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
				if (ConcreteVectorMessage.IsValid())
				{
					const float x = ConcreteVectorMessage->x;
					const float y = ConcreteVectorMessage->y;
					const float z = ConcreteVectorMessage->z;
					FVector Data(x,y,z);
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Data, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnVector3Message(Data);
					});
				}
				break;
			}
			case EMessageType::Point:
			{
				auto ConcreteVectorMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::Point>(msg);
				if (ConcreteVectorMessage.IsValid())
				{
					const float x = ConcreteVectorMessage->x;
					const float y = ConcreteVectorMessage->y;
					const float z = ConcreteVectorMessage->z;
					FVector Data(x, y, z);
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Data, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnPointMessage(Data);
					});
				}
				break;
			}
			case EMessageType::Pose:
			{
				auto ConcreteVectorMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(msg);
				if (ConcreteVectorMessage.IsValid())
				{
					const auto p = ConcreteVectorMessage->position;
					const auto q = ConcreteVectorMessage->orientation;
					const FVector  position(p.x, p.y, p.z);
					const FVector4 orientation(q.x,q.y,q.z,q.w);
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, position, orientation, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnPoseMessage(position, orientation);
					});
				}
				break;
			}
			case EMessageType::Quaternion:
			{
				auto ConcreteVectorMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::Quaternion>(msg);
				if (ConcreteVectorMessage.IsValid())
				{
					const auto q = ConcreteVectorMessage;
					const FVector4 orientation(q->x, q->y, q->z, q->w);
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, orientation, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnQuaternionMessage(orientation);
					});
				}
				break;
			}
			case EMessageType::Twist:
			{
				auto ConcreteVectorMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::Twist>(msg);
				if (ConcreteVectorMessage.IsValid())
				{
					const auto p = ConcreteVectorMessage->linear;
					const auto q = ConcreteVectorMessage->angular;
					const FVector  linear(p.x, p.y, p.z);
					const FVector angular(q.x, q.y, q.z);
					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, linear, angular, SelfPtr]()
					{
						if (!SelfPtr.IsValid()) return;
						OnTwistMessage(linear, angular);
					});
				}
				break;
			}
			default:
				unimplemented();
				break;
			}
		};

		success = Subscribe(Callback);
	}

	return success;
}


bool UTopic::PublishStringMessage(const FString& Message)
{
	check(_Implementation->_MessageType == TEXT("std_msgs/String"));

	if (!_State.Advertised)
	{
		if (!Advertise())
		{
			return false;
		}
	}

	TSharedPtr<ROSMessages::std_msgs::String> msg = MakeShareable(new ROSMessages::std_msgs::String);
	msg->_Data = Message;
	return _Implementation->Publish(msg);
}
