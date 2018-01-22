#pragma once


#include <iostream>
//#include <stdio.h>
//#include <string>
//#include <chrono>
#include <thread>

#include <functional> // std::function

// #include "json.hpp"
//#include "CoreMinimal.h"
#include "CoreMinimal.h"
//#include "ThreadingBase.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
//#include "Networking.h" // Unreal networking


#include "itransport_layer.h"
#include "types.h"
//

#include "rapidjson/document.h"
using json = rapidjson::Document;

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"


#pragma warning(disable:4265)
class TCPConnection : public rosbridge2cpp::ITransportLayer{
public:
  TCPConnection (){
	  UE_LOG(LogTemp, Warning, TEXT("TCPConnection Constructor"));
  }
  ~TCPConnection (){
	UE_LOG(LogTemp, Warning, TEXT("Connection Destructor called now"));

    if(_sock!=nullptr){
      UE_LOG(LogTemp, Warning, TEXT("Socket is not nullptr. Closing ..."));
      _sock->Close();
    }
    terminateReceiverThread = true;
	UE_LOG(LogTemp, Warning, TEXT("Waiting for Thread to join"));
    if(receiverThreadSetUp){
      receiverThread.join(); // Wait for the receiver thread to finish
	  UE_LOG(LogTemp, Warning, TEXT("join() in Connection Destructor done"));
    }else{
	  UE_LOG(LogTemp, Warning, TEXT("receiverThread hasn't been set up. Skipping join() on it"));
    }
  }

  bool Init(std::string ip_addr, int port);
  bool SendMessage(std::string data);
  bool SendMessage(const uint8_t *data, unsigned int length);
  uint16_t Fletcher16( const uint8_t *data, int count );
  int ReceiverThreadFunction();
  void RegisterIncomingMessageCallback(std::function<void(json&)> fun);
  void RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun);
  void RegisterErrorCallback(std::function<void(rosbridge2cpp::TransportError)> fun);
  void ReportError(rosbridge2cpp::TransportError err);
  void SetTransportMode(rosbridge2cpp::ITransportLayer::TransportMode);


private:
  std::string _ip_addr;
  int _port;

  FSocket *_sock = nullptr; 
  // int sock = socket(AF_INET , SOCK_STREAM , 0);
  // struct sockaddr_in connect_to;
  std::thread receiverThread;
  bool terminateReceiverThread = false;
  bool receiverThreadSetUp = false;
  bool _callback_function_defined = false;
  bool bson_only_mode_ = false;
  std::function<void(json&)> _incoming_message_callback; 
  std::function<void(bson_t&)> incoming_message_callback_bson_; 
  std::function<void(rosbridge2cpp::TransportError)> _error_callback;
};
#pragma warning(default:4265)
