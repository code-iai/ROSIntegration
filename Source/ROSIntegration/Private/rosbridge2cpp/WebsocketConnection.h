#pragma once

#include <functional> // std::function

#include <CoreMinimal.h>

//#include Websocket stuff
#include <SocketSubsystem.h>
#include <Sockets.h>
#include "itransport_layer.h"
#include "types.h"
//

#include "rapidjson/document.h"
using json = rapidjson::Document;

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "IWebSocket.h"

#pragma warning(disable:4265)
class WebsocketConnection : public rosbridge2cpp::ITransportLayer {
public:
	WebsocketConnection();	//constructor
	~WebsocketConnection(); //deconstructor


	bool Init(std::string ip_addr, int port);
	bool SendMessage(std::string data);
	bool SendMessage(const uint8_t* data, unsigned int length);
	void RegisterIncomingMessageCallback(std::function<void(json&)> fun);
	void RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun);
	void RegisterErrorCallback(std::function<void(rosbridge2cpp::TransportError)> fun);
	void ReportError(rosbridge2cpp::TransportError err);
	void SetTransportMode(rosbridge2cpp::ITransportLayer::TransportMode);

	bool IsHealthy() const;

private:
	void OnConnectionError(const FString& Error);
	void OnClosed(int32 StatusCode, const FString& Reason, bool bWasClean);
	void OnRawMessage(const void*, size_t size, size_t bytes_remaining);
	void OnMessage(const FString& msg);


	TSharedPtr<IWebSocket> WebSocket;

	bool bson_only_mode_ = false;
	std::function<void(json&)> _incoming_message_callback;
	std::function<void(bson_t&)> incoming_message_callback_bson_;
	std::function<void(rosbridge2cpp::TransportError)> _error_callback;
};
#pragma warning(default:4265)
