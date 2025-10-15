#pragma once

#include <functional> // std::function

#include <CoreMinimal.h>
#include "IWebSocket.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
// #include <openssl/sha.h>   // for SHA1
// #include <openssl/bio.h>   // for Base64
// #include <openssl/evp.h>
#pragma comment(lib, "ws2_32.lib")

//#include Websocket stuff
#include "itransport_layer.h"
#include "types.h"
#include <thread>
#include "rapidjson/document.h"
using json = rapidjson::Document;

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "IWebSocket.h"

class WebsocketOverride : public IWebSocket
{
public:
	// class meant to take the place of IWebSocket, since it sucks
	WebsocketOverride(std::string ip_address, int port_num);
	virtual ~WebsocketOverride() override;
	
	void Init();
	// main thing you want it to do is connect

	virtual void Connect() override;
	
	// Implement all IWebSocket pure virtual methods as functions
	virtual void Close(int32 Code, const FString& Reason) override {}
	// can probably just consolidate this into Close
	void NewClose(int32 Code, const FString& Reason);
	// check handle stored in SOCKET (may need to store as a field)
	virtual bool IsConnected() override;
	// actually sending topic to subscribe or data?
	virtual void Send(const FString& Data) override;
	virtual void Send(const void* Data, SIZE_T Size, bool bIsBinary) override;
	virtual void SetTextMessageMemoryLimit(uint64 TextMessageMemoryLimit) override {}

	virtual FWebSocketConnectedEvent& OnConnected() override {
		static FWebSocketConnectedEvent Dummy; 
		return Dummy;
	}

	virtual FWebSocketConnectionErrorEvent& OnConnectionError() override {
		static FWebSocketConnectionErrorEvent Dummy; 
		return Dummy;
	}

	virtual FWebSocketClosedEvent& OnClosed() override {
		static FWebSocketClosedEvent Dummy; 
		return Dummy;
	}

	// need to implememnt on Message and on Binary message in order to actually receive messages
	virtual FWebSocketMessageEvent& OnMessage() override {
		return MessageEvent;
	}

	virtual FWebSocketBinaryMessageEvent& OnBinaryMessage() override {
		return BinaryMessageEvent;
	}

	virtual FWebSocketRawMessageEvent& OnRawMessage() override {
		return RawMessageEvent;
	}

	virtual FWebSocketMessageSentEvent& OnMessageSent() override {
		return MessageSentEvent;
	}

	// thread that continually calls recv to check for new messages
	void ReceiveMessageLoop();

	
	bool checkConnection() const;
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
	FString URI;
	std::string URIString;
	bool isConnected;
	SOCKET socketStorage;
	FWebSocketMessageEvent MessageEvent;
	FWebSocketBinaryMessageEvent BinaryMessageEvent;
	FWebSocketRawMessageEvent RawMessageEvent;
	FWebSocketMessageSentEvent MessageSentEvent;

	// thread safe bool, whether of not receive check thread is running
	std::atomic<bool> thread_run;
	// thread that continually checks for incoming message
	std::thread receive_thread;

	TSharedPtr<IWebSocket> WebSocket;
	std::string ip_addr;
	int port;

	bool bson_only_mode_ = false;
	std::function<void(json&)> _incoming_message_callback;
	std::function<void(bson_t&)> incoming_message_callback_bson_;
	std::function<void(rosbridge2cpp::TransportError)> _error_callback;
	
};
