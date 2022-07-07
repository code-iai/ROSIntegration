#include "RI/Topic.h"
#include <bson.h>
#include "rosbridge2cpp/ros_bridge.h"
#include "rosbridge2cpp/ros_topic.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"

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
		SupportedMessageTypes.Add(EMessageType::String, TEXT("std_msgs/String"));
		SupportedMessageTypes.Add(EMessageType::Float32, TEXT("std_msgs/Float32"));
		SupportedMessageTypes.Add(EMessageType::PoseStamped, TEXT("geometry_msgs/PoseStamped"));
		SupportedMessageTypes.Add(EMessageType::TwistStamped, TEXT("geometry_msgs/TwistStamped"));
		SupportedMessageTypes.Add(EMessageType::Twist, TEXT("geometry_msgs/Twist"));
		SupportedMessageTypes.Add(EMessageType::HomePosition, TEXT("mavros_msgs/HomePosition"));
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
	return msg != nullptr && _State.Connected && _Implementation->Publish(msg);
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
			case EMessageType::PoseStamped:
			{
				auto ConcretePoseStampedMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
				if (ConcretePoseStampedMessage.IsValid())
				{
					const FVector Pos = FVector(
						ConcretePoseStampedMessage->pose.position.x * 100,
						ConcretePoseStampedMessage->pose.position.y * 100,
						ConcretePoseStampedMessage->pose.position.z * -100);

					const FQuat Quat = FQuat(
						ConcretePoseStampedMessage->pose.orientation.x,
						ConcretePoseStampedMessage->pose.orientation.y * -1,
						ConcretePoseStampedMessage->pose.orientation.w * -1,
						ConcretePoseStampedMessage->pose.orientation.z * -1);

					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Pos, Quat, SelfPtr]()
						{
							if (!SelfPtr.IsValid()) return;
							FRotator Rot = Quat.Rotator();
							Rot.Yaw -= 90;
							float temp = Rot.Pitch;
							Rot.Pitch = Rot.Roll;
							Rot.Roll = temp*-1;
							OnPoseStampedMessage(Pos, Rot);
						});
				}
				break;
			}
			case EMessageType::TwistStamped:
			{
				auto ConcreteTwistStampedMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::TwistStamped>(msg);
				if (ConcreteTwistStampedMessage.IsValid())
				{
					const FVector Linear = FVector(
						ConcreteTwistStampedMessage->twist.linear.x * 100,
						ConcreteTwistStampedMessage->twist.linear.y * 100,
						ConcreteTwistStampedMessage->twist.linear.z * 100);

					const FVector Angular = FVector(
						ConcreteTwistStampedMessage->twist.angular.x,
						ConcreteTwistStampedMessage->twist.angular.y * -1,
						ConcreteTwistStampedMessage->twist.angular.z);

					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Linear, Angular, SelfPtr]()
						{
							if (!SelfPtr.IsValid()) return;
							OnTwistStampedMessage(Linear, Angular);
						});
				}
				break;
			}
			case EMessageType::Twist:
			{
				auto ConcreteTwistMessage = StaticCastSharedPtr<ROSMessages::geometry_msgs::Twist>(msg);
				if (ConcreteTwistMessage.IsValid())
				{
					const FVector Linear = FVector(
						ConcreteTwistMessage->linear.x * 100,
						ConcreteTwistMessage->linear.y * -100,
						ConcreteTwistMessage->linear.z * 100);

					const FVector Angular = FVector(
						ConcreteTwistMessage->angular.x,
						ConcreteTwistMessage->angular.y * -1,
						ConcreteTwistMessage->angular.z);

					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Linear, Angular, SelfPtr]()
						{
							if (!SelfPtr.IsValid()) return;
							OnTwistMessage(Linear, Angular);
						});
				}
				break;
			}

			case EMessageType::HomePosition:
			{
				auto ConcreteHomePositionMessage = StaticCastSharedPtr<ROSMessages::mavros_msgs::HomePosition>(msg);
				if (ConcreteHomePositionMessage.IsValid())
				{
					const FVector Geo = FVector(
						ConcreteHomePositionMessage->geo.latitude,
						ConcreteHomePositionMessage->geo.longitude,
						ConcreteHomePositionMessage->geo.altitude);

					const FVector Position = FVector(
						ConcreteHomePositionMessage->position.x * 100,
						ConcreteHomePositionMessage->position.y * 100,
						ConcreteHomePositionMessage->position.z * -100);

					const FQuat Orientation = FQuat(
						ConcreteHomePositionMessage->orientation.x,
						ConcreteHomePositionMessage->orientation.y * -1,
						ConcreteHomePositionMessage->orientation.w * -1,
						ConcreteHomePositionMessage->orientation.z * -1);

					const FVector Approach = FVector(
						ConcreteHomePositionMessage->approach.x,
						ConcreteHomePositionMessage->approach.y,
						ConcreteHomePositionMessage->approach.z);

					TWeakPtr<UTopic, ESPMode::ThreadSafe> SelfPtr(_SelfPtr);
					AsyncTask(ENamedThreads::GameThread, [this, Geo, Position, Orientation, Approach, SelfPtr]()
						{
							FRotator Rot = Orientation.Rotator();
							Rot.Yaw += 180;

							if (!SelfPtr.IsValid()) return;
							OnHomePositionMessage(Geo, Position, Rot, Approach);
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



bool UTopic::PublishTwistStampedMessage(const FVector& Linear, const FVector& Angular)
{

	if (!_State.Advertised)
	{
		if (!Advertise())
		{
			return false;
		}
	}

	TSharedPtr<ROSMessages::geometry_msgs::TwistStamped> msg = MakeShareable(new ROSMessages::geometry_msgs::TwistStamped);
	msg->twist.linear = Linear;
	msg->twist.linear.y *= -1;
	msg->twist.angular = Angular;
	msg->twist.angular.z *= -1;
	msg->twist.angular.y *= -1;

	return _Implementation->Publish(msg);
}


bool UTopic::PublishTwistMessage(const FVector& Linear, const FVector& Angular)
{

	if (!_State.Advertised)
	{
		if (!Advertise())
		{
			return false;
		}
	}

	TSharedPtr<ROSMessages::geometry_msgs::Twist> msg = MakeShareable(new ROSMessages::geometry_msgs::Twist);
	msg->linear = Linear;
	float temp = msg->linear.y;
	msg->linear.y = msg->linear.x;
	msg->linear.x = temp;
	msg->angular = Angular;
	msg->angular.z *= -1;
	return _Implementation->Publish(msg);
}