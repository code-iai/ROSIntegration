#include "WebsocketOverride.h"
#include <random>
#include <vector>
// #include <openssl/bio.h>
// #include <openssl/evp.h>
#include "ROSIntegrationCore.h"

WebsocketOverride::WebsocketOverride(std::string ip_address, int port_num) : URI(""),
isConnected(false),
port(port_num),
thread_run(false),
ip_addr(ip_address),
socketStorage(INVALID_SOCKET)
{
	Init();
}

WebsocketOverride::~WebsocketOverride() {
	NewClose(1000, TEXT("Websocket Closing, Object Being Destroyed"));
};


/**
 * Called in the destructor in order to clean up the Websocket. Closes the socket if it's open and
 * joins the receive_thread (checks for published data to ws)
 * @param Code Whether or not it's being closed cleanly or via error (right now just hard coded to 1000,
 * TODO: Either get rid of code or make it change based upon what situation it's being called
 * @param Reason For UE_LOG, explains context that it's closing
 */
void WebsocketOverride::NewClose(int32 Code, const FString& Reason)
{
	thread_run = false;
	// joins thread back to main
	if (receive_thread.joinable())
		receive_thread.join();
	// closes socket if it's open
	if (isConnected && socketStorage != INVALID_SOCKET)
	{
		UE_LOG(LogROS, Display, TEXT("Closing socket connection..."));
		closesocket(socketStorage);
		socketStorage = INVALID_SOCKET;
		isConnected = false;
	} else
	{
		UE_LOG(LogROS, Warning, TEXT("Websocket not closing because there isn't one open"));
	}
	UE_LOG(LogROS, Display, TEXT("%s"), *Reason);
}

/**
 * 
 * @return whether you're currently connected to the websocket
 */
bool WebsocketOverride::checkConnection() const
{
	return isConnected;
}

/**
 * TODO: Simplify so you only use one of these methods (See which one is actually being called)
 * @return Calls check connection
 */
bool WebsocketOverride::IsConnected()
{
	return checkConnection();
}

/**
 * Checks if the handshake with the websocket was succesful
 * @param sock Handle reference to the socket that you're connected to
 * @return whether the handshake connecting to the server was successful
 */
bool checkHandshake(SOCKET& sock)
{
	// stores response from server
	char buffer[40960];
	// recv call receives in the header from the server
	int bytes_received = recv(sock, buffer, sizeof(buffer) - 1, 0);
	if (bytes_received > 0)
	{
		buffer[bytes_received] = '\0';
		std::string response(buffer);
		UE_LOG(LogROS, Display, TEXT("Handshake accepted, server response: %s"), *FString(response.c_str()));
		return true;
	} else
	{
		UE_LOG(LogROS, Error, TEXT("Handshake with server failed"));
		return false;
	}
}

// generates a random base 64 encoded key to use for handshake
/**
 * TODO: make actually generate a random key and not just a hard coded one
 * @return random key to send in ws header
 */
std::string generate_random_key()
{
	return "dGhlIHNhbXBsZSBub25jZQ==";
}

/**
 * TODO: just make this a member function so you don't have to pass in member fields
 * @param ip_address ip address of the socket you want to connect to
 * @param port_num port number you're connecting to
 * @param uri_string full combo of ip address and port number (could just create this in the function)
 * @param sock Socket handle
 * @return Whether the connection was successful
 */
bool SocketCreate(const std::string& ip_address, const int& port_num, const std::string& uri_string, SOCKET& sock)
{
	// creates the socket to connect to the rosbridge server
	// gives you the handle you need
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock == INVALID_SOCKET) {
		UE_LOG(LogROS, Error, TEXT("Failed to create socket"));
	}
	// specifies your IPv4 address
	sockaddr_in serverAddress;
	// sets port, family type, and server address
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(port_num);
	inet_pton(AF_INET, ip_address.c_str(), &serverAddress.sin_addr);
	// the actual TCP connection being called 
	if (connect(sock, (sockaddr*)&serverAddress, sizeof(serverAddress)) < 0)
	{
		UE_LOG(LogROS, Error, TEXT("Failed to connect to TCP server"));
		return false;
	} else
	{
		UE_LOG(LogROS, Display, TEXT("Connected to TCP server!"));
		std::string key = generate_random_key();
		
		// handshake request message, upgrading to websocket connection
		std::ostringstream request;
		request << "GET / HTTP/1.1\r\n";
		request << "Host: " << uri_string << "\r\n";
		request << "Upgrade: websocket\r\n";
		request << "Connection: Upgrade\r\n";
		request << "Sec-WebSocket-Key: " << key << "\r\n";
		request << "Sec-WebSocket-Version: 13\r\n\r\n";

		std::string reqStr = request.str();
		// sends the actual http upgrade request
		send(sock, reqStr.c_str(), reqStr.size(), 0);

		// checking whether or not the handshake (http upgrade request) was successful
		return checkHandshake(sock);
	}
	
}

/**
 * Helper function called in constructor, creates and saves the URI to be used in ws connection
 */
void WebsocketOverride::Init()
{
	TArray< FStringFormatArg > args;
	args.Add(FStringFormatArg(ip_addr.c_str()));
	args.Add(FStringFormatArg(port));
	const FString ServerURL = FString::Format(TEXT("ws://{0}:{1}"), args);
	
	UE_LOG(LogROS, Display, TEXT("URI: %s"), *ServerURL);
	URI = ServerURL;
	URIString = ip_addr + ":" + std::to_string(port);
}

/**
 * Called to connect to websocket, uses socketCreate as helper
 * Also dispatches thread to continually loop and check for messages from the connection
 */
void WebsocketOverride::Connect()
{
	isConnected = SocketCreate(ip_addr, port, URIString, socketStorage);
	if (isConnected)
	{
		UE_LOG(LogROS, Display, TEXT("Dispatching receive loop thread"));
		// sets non-blocking mode for the socket if recv call fails
		u_long thread_mode = 1;
		ioctlsocket(socketStorage, FIONBIO, &thread_mode);
		
		thread_run = true;
		// dispatch thread that continually loops and checks for message updates
		receive_thread = std::thread(&WebsocketOverride::ReceiveMessageLoop, this);
	}
}


/**
 * Send for string data
 * @param Data String you want to send over the ws connection
 */
void WebsocketOverride::Send(const FString& Data)
{
	UE_LOG(LogROS, Display, TEXT("Calling send string from websocket override"));
	std::string utf8 = TCHAR_TO_UTF8(*Data);
	SendMessage(utf8);
}

// for sending binary to server
/**
 * Send for binary data (don't really use this but needed to implement in case it gets called somewhere i don't know about)
 * @param Data Data being sent over
 * @param Size size of data
 * @param bIsBinary whether data is binary (should be or this wouldn't be getting called)
 */
void WebsocketOverride::Send(const void* Data, SIZE_T Size, bool bIsBinary)
{
	UE_LOG(LogROS, Display, TEXT("Calling send binary from websocket override"));
	SendMessage(reinterpret_cast<const uint8_t*>(Data), static_cast<unsigned int>(Size));
	
}

/**
 * helper for Send function, actually sends string data over the ws connection
 * @param data string of data to send
 * @return whether send was successful
 */
bool WebsocketOverride::SendMessage(std::string data)
{
	UE_LOG(LogROS, Display, TEXT("Calling send message string from websocket override"));

	if (!isConnected || socketStorage == INVALID_SOCKET)
	{
		UE_LOG(LogROS, Error, TEXT("Can't send message, not connected to socket"));
		return false;
	}
	UE_LOG(LogROS, Display, TEXT("Message being sent from string SendMessage to server: %hs"), data.c_str());

	// frame to actually put encoding in so it's ready to send to the ws
	std::vector<uint8_t> frame;
	// opcode for text
	uint8_t opcode = 0x1;
	// marks final frame and the opcode for it
	frame.push_back(0x80 | opcode);

	
	size_t length = data.size();
	if (length <= 125) {
		frame.push_back(0x80 | static_cast<uint8_t>(length));
	} else if (length <= 65535) {
		frame.push_back(0x80 | 126);
		frame.push_back((length >> 8) & 0xFF);
		frame.push_back(length & 0xFF);
	} else {
		frame.push_back(0x80 | 127);
		for (int i = 7; i >= 0; i--) {
			frame.push_back((length >> (i * 8)) & 0xFF);
		}
	}

	// Mask key (4 random bytes)
	uint8_t maskKey[4];
	for (int i = 0; i < 4; i++) maskKey[i] = rand() % 256;
	frame.insert(frame.end(), maskKey, maskKey + 4);

	// Mask the payload
	for (size_t i = 0; i < length; i++) {
		frame.push_back(static_cast<uint8_t>(data[i]) ^ maskKey[i % 4]);
	}

	// Send the frame over TCP
	int totalSent = 0;
	const char* frameData = reinterpret_cast<const char*>(frame.data());
	int frameSize = static_cast<int>(frame.size());

	while (totalSent < frameSize)
	{
		// not the same send call as the one in websocket class, not an infinite loop, chill
		int sent = send(socketStorage, frameData + totalSent, frameSize - totalSent, 0);
		if (sent == SOCKET_ERROR)
		{
			int lastError = WSAGetLastError();
			UE_LOG(LogROS, Error, TEXT("WebSocket Send failed, WSA error %d"), lastError);
			return false;
		}
		totalSent += sent;
	}

	return true;
}

/**
 * Send message but for binary data (once again not really used but just in case you need to send binary)
 * @param data binary data to send
 * @param length size of data
 * @return 
 */
bool WebsocketOverride::SendMessage(const uint8_t* data, unsigned int length)
{
	UE_LOG(LogROS, Display, TEXT("Calling send message binary from websocket override"));
	if (!isConnected || socketStorage == INVALID_SOCKET)
	{
		UE_LOG(LogROS, Error, TEXT("Can't send message, not connected to socket"));
		return false;
	}

	UE_LOG(LogROS, Display, TEXT("Sending message from binary"));
	UE_LOG(LogROS, Display, TEXT("Sending length: %u data:%hhu "), length, *data);

	// create websocket frame to send package in

	// frame to put data in
	std::vector<uint8_t> frame;

	// first byte of frame header, tells you it's a binary header
	uint8_t opcode = 0x2;
	// sets final frame marker, as well as opcode
	frame.push_back(0x80 | opcode);
	

	// second byte of frame header, tells how long your frame is
	// Bit 1: mask, bits 2-8 payload length
	if (length <= 125)
	{
		// store length in 7 bits
		frame.push_back(0x80 | static_cast<uint8_t>(length));
	} else if (length <= 65535) {
		frame.push_back(0x80 | 126);
		frame.push_back((length >> 8) & 0xFF);
		frame.push_back(length & 0xFF);
	} else {
		frame.push_back(0x80 | 127);
		for (int i = 7; i >= 0; i--) {
			frame.push_back((length >> (i * 8)) & 0xFF);
		}
	}

	// Masking key (4 random bytes)
	uint8_t maskKey[4];
	for (int i = 0; i < 4; i++) maskKey[i] = rand() % 256;
	frame.insert(frame.end(), maskKey, maskKey + 4);

	// Mask the payload
	for (unsigned int i = 0; i < length; i++) {
		frame.push_back(data[i] ^ maskKey[i % 4]);
	}

	// Send frame over socket
	int totalSent = 0;
	const char* frameData = reinterpret_cast<const char*>(frame.data());
	int frameSize = static_cast<int>(frame.size());

	while (totalSent < frameSize) {
		int sent = send(socketStorage, frameData + totalSent, frameSize - totalSent, 0);

		if (sent == SOCKET_ERROR) {
			int err = WSAGetLastError();
			UE_LOG(LogTemp, Error, TEXT("WebSocket Send failed, WSA error: %d"), err);

			if (err == WSAEWOULDBLOCK) {
				continue; // try again
			}
			return false;
		}

		totalSent += sent;
	}

	return true;
}

void WebsocketOverride::RegisterIncomingMessageCallback(std::function<void(json&)> fun)
{
	_incoming_message_callback = fun;
}

void WebsocketOverride::RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun)
{
	incoming_message_callback_bson_ = fun;
}

void WebsocketOverride::RegisterErrorCallback(std::function<void(rosbridge2cpp::TransportError)> fun)
{
	_error_callback = fun;
}

void WebsocketOverride::ReportError(rosbridge2cpp::TransportError err)
{
	if (_error_callback)
		_error_callback(err);
}

void WebsocketOverride::SetTransportMode(rosbridge2cpp::ITransportLayer::TransportMode mode)
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

bool WebsocketOverride::IsHealthy() const
{
	return isConnected;
}


/**
 * Continually polls for updates on the ws connection. If there is new changes, triggers callback function
 * Called by the receive thread
 */
void WebsocketOverride::ReceiveMessageLoop()
{
	UE_LOG(LogROS, Display, TEXT("Calling receive message loop from thread"));
	// constexpr means value known at compile time, as opposed to regular const
	// important because you're allocating size of buffer at compile time
	constexpr size_t BUFFER_SIZE = 8192;
	// stores your incoming data from the socket
	uint8_t buffer[BUFFER_SIZE];
	
	while (isConnected && thread_run)
	{
		// checking to see if there is any messages being sent over from server
		int received_bytes = recv(socketStorage,reinterpret_cast<char*>(buffer), BUFFER_SIZE, 0);
		if (received_bytes > 0)
		{
			if (bson_only_mode_)
			{
				// binary incoming, stores data from buffer as a bson_t
				bson_t b;
				if (!bson_init_static(&b, buffer, received_bytes))
				{
					UE_LOG(LogROS, Error, TEXT("Error parsing BSON in receive thread"));
				}
				else if (incoming_message_callback_bson_)
				{
					incoming_message_callback_bson_(b);
				}
			}
			else
			{
				// json mode, registers incoming message as a json
				FString msgStr = UTF8_TO_TCHAR(reinterpret_cast<const char*>(buffer));
				UE_LOG(LogROS, Display, TEXT("Received json message: %s"), *msgStr);
				OnMessage(msgStr);
			}
		} else if (received_bytes == 0)
		{
			UE_LOG(LogROS, Warning, TEXT("Socket closed by remote"));
			thread_run = false;
			isConnected = false;
			break;
		}
		else
		{
			int err = WSAGetLastError();
			if (err != WSAEWOULDBLOCK)
			{
				UE_LOG(LogROS, Error, TEXT("Socket recv error: %d"), err);
				thread_run = false;
				isConnected = false;
				break;
			}
			// adjust if you want more time in between publish checks (if the game starts to lag)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		
	}
	
}


/**
 * On updated data being sent
 * @param msg new data sent
 */
void WebsocketOverride::OnMessage(const FString& msg) {
	json j;
	UE_LOG(LogROS, Display, TEXT("Received message, calling OnMessage"));
	try {
		j.Parse(TCHAR_TO_ANSI(*msg)); //convert to UTF-8 and then reinterpret_cast the pointer to char *

		if (_incoming_message_callback)
			_incoming_message_callback(j);
	}
	catch (...) {
		UE_LOG(LogROS, Error, TEXT("Failed to parse JSON - Ignoring message"));
	}
}


void WebsocketOverride::OnRawMessage(const void* data, size_t size, size_t bytes_remaining)
{
	UE_LOG(LogROS, Display, TEXT("Received raw message, calling OnRawMessage"));
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

//function void IWebSocket::Close(int32 Code, const FString& Reason) is pure virtual
//	function bool IWebSocket::IsConnected() is pure virtual
//	function void IWebSocket::Send(const FString& Data) is pure virtual
//	function void IWebSocket::Send(const void* Data, SIZE_T Size, bool bIsBinary) is pure virtual
//	function void IWebSocket::SetTextMessageMemoryLimit(uint64 TextMessageMemoryLimit) is pure virtual
//	function IWebSocket::FWebSocketConnectedEvent& IWebSocket::OnConnected() is pure virtual
//	function IWebSocket::FWebSocketConnectionErrorEvent& IWebSocket::OnConnectionError() is pure virtual
//	function IWebSocket::FWebSocketClosedEvent& IWebSocket::OnClosed() is pure virtual
//	function IWebSocket::FWebSocketMessageEvent& IWebSocket::OnMessage() is pure virtual
//	function IWebSocket::FWebSocketBinaryMessageEvent& IWebSocket::OnBinaryMessage() is pure virtual
//	function IWebSocket::FWebSocketRawMessageEvent& IWebSocket::OnRawMessage() is pure virtual
//	function IWebSocket::FWebSocketMessageSentEvent& IWebSocket::OnMessageSent() is pure virtual