#include "WebsocketConnection.h"
#include <iostream>
#include "ROSIntegrationCore.h"
#include "WebSocketsModule.h"  //moduel used to create websocket object in init 
#include <iomanip>

#include <Interfaces/IPv4/IPv4Address.h>
#if ENGINE_MINOR_VERSION < 23
#include <IPAddress.h>
#endif
#include <Serialization/ArrayReader.h>
#include <SocketSubsystem.h>

WebsocketConnection::WebsocketConnection() : _incoming_message_callback(nullptr), incoming_message_callback_bson_(nullptr), _error_callback(nullptr) {
}

WebsocketConnection::~WebsocketConnection() {
	WebSocket->OnConnectionError().RemoveAll(this);
	WebSocket->OnClosed().RemoveAll(this);
	WebSocket->OnMessage().RemoveAll(this);
	WebSocket->OnRawMessage().RemoveAll(this);
	ReportError(rosbridge2cpp::TransportError::R2C_CONNECTION_CLOSED);
	WebSocket->Close();
}

bool WebsocketConnection::Init(std::string ip_addr, int port)
{

	//LWS throttles messages by default, check to see if it has been overridden
	double ThreadTargetFrameTimeInSeconds = 0.0;
	bool setGlobally = GConfig->GetDouble(TEXT("WebSockets.LibWebSockets"), TEXT("ThreadTargetFrameTimeInSeconds"), ThreadTargetFrameTimeInSeconds, GEngineIni);

	if (!FModuleManager::Get().IsModuleLoaded("WebSockets"))
	{
		//If throttling not overridden by the .ini, set to 0 before initializing the module
		if (!setGlobally) {
			GConfig->SetDouble(TEXT("WebSockets.LibWebSockets"), TEXT("ThreadTargetFrameTimeInSeconds"), 0.0, GEngineIni);
		} else if (ThreadTargetFrameTimeInSeconds > 0.0) {
			UE_LOG(LogROS, Warning, TEXT("WebSockets.LibWebSockets.ThreadTargetFrameTimeInSeconds is set to be > 0.0. You may experience ROS lag."));
		}

		FModuleManager::Get().LoadModule("Websockets");
	} else {
		//Sanity check thread throttling
		if (!setGlobally || ThreadTargetFrameTimeInSeconds > 0.0) {
			UE_LOG(LogROS, Warning, TEXT("WebSockets.LibWebSockets.ThreadTargetFrameTimeInSeconds is either not set, or set to be > 0.0. You may experience ROS lag."));
		}
	}


	//Connect to Websocket
	TArray< FStringFormatArg > args;
	args.Add(FStringFormatArg(ip_addr.c_str()));
	args.Add(FStringFormatArg(port));
	const FString ServerURL = FString::Format(TEXT("ws://{0}:{1}"), args);
	UE_LOG(LogROS, Display, TEXT("Connecting to: %s"), *ServerURL);

	WebSocket = FWebSocketsModule::Get().CreateWebSocket(ServerURL);

	//Set up callbacks
	WebSocket->OnConnectionError().AddRaw(this, &WebsocketConnection::OnConnectionError);
	WebSocket->OnClosed().AddRaw(this, &WebsocketConnection::OnClosed);
	WebSocket->OnMessage().AddRaw(this, &WebsocketConnection::OnMessage);
	WebSocket->OnRawMessage().AddRaw(this, &WebsocketConnection::OnRawMessage);

	WebSocket->Connect();

	return true;
}

void WebsocketConnection::OnConnectionError(const FString& Error) {
	UE_LOG(LogROS, Display, TEXT("WebSocket Connect Error"));
	ReportError(rosbridge2cpp::TransportError::R2C_SOCKET_ERROR);
}

void WebsocketConnection::OnClosed(int32 StatusCode, const FString& Reason, bool bWasClean) {
	UE_LOG(LogROS, Warning, TEXT("Connection closed: %d:%s. Clean: %d"), StatusCode, *Reason, bWasClean);
	ReportError(rosbridge2cpp::TransportError::R2C_CONNECTION_CLOSED);
}

bool WebsocketConnection::SendMessage(std::string data)
{
	//Data is already UTF-8, no conversion to UE4 string is necessary
	WebSocket->Send(data.c_str(), data.size(), false);
	return true;
}

bool WebsocketConnection::SendMessage(const uint8_t* data, unsigned int length)
{
	WebSocket->Send(data, length, true);
	return true;
}

void WebsocketConnection::OnRawMessage(const void* data, size_t size, size_t bytes_remaining)
{
	if (bytes_remaining > 0) {
		UE_LOG(LogROS, Warning, TEXT("Websocket message fragments not supported - Ignoring message"));
		return;
	}
	bson_t b;
	if (!bson_init_static(&b, reinterpret_cast<const uint8_t*>(data), size)) {
		UE_LOG(LogROS, Error, TEXT("Error on BSON parse - Ignoring message"));
	} else if (incoming_message_callback_bson_) {
		incoming_message_callback_bson_(b);
	}
}

void WebsocketConnection::OnMessage(const FString& msg) {
	json j;
	try {
		j.Parse(TCHAR_TO_ANSI(*msg)); //convert to UTF-8 and then reinterpret_cast the pointer to char *

		if (_incoming_message_callback)
			_incoming_message_callback(j);
	}
	catch (...) {
		UE_LOG(LogROS, Error, TEXT("Failed to parse JSON - Ignoring message"));
	}
}

void WebsocketConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun)
{
	_incoming_message_callback = fun;
}

void WebsocketConnection::RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun)
{
	incoming_message_callback_bson_ = fun;
}

void WebsocketConnection::RegisterErrorCallback(std::function<void(rosbridge2cpp::TransportError)> fun)
{
	_error_callback = fun;
}

void WebsocketConnection::ReportError(rosbridge2cpp::TransportError err)
{
	if (_error_callback)
		_error_callback(err);
}


void WebsocketConnection::SetTransportMode(rosbridge2cpp::ITransportLayer::TransportMode mode)
{
	switch (mode) {
	case rosbridge2cpp::ITransportLayer::JSON:
		bson_only_mode_ = false;
		break;
	case rosbridge2cpp::ITransportLayer::BSON:
		bson_only_mode_ = true;
		break;
	default:
		UE_LOG(LogROS, Error, TEXT("Given TransportMode not implemented!"));
	}
}

bool WebsocketConnection::IsHealthy() const
{
	return WebSocket->IsConnected();
}
