#include "TCPConnection.h"

//#include "Runtime/Networking/Public/Interfaces/IPv4/IPv4Address.h"
//#include "Runtime/Networking/Public/Common/TcpSocketBuilder.h"
#include "Networking.h"

#include <iomanip>

// void messageCallback(const json &message){
//       std::string pkg_op = message["op"];
//       std::cout << "Type of received message: " << pkg_op;
// }

bool TCPConnection::Init(std::string ip_addr, int port) {

  FString address = FString(ip_addr.c_str());
  int32 remote_port = port;
  FIPv4Address ip;
  FIPv4Address::Parse(address, ip);

  auto addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
  bool ipValid = false;
  addr->SetIp(*address, ipValid);
  addr->SetPort(remote_port);

  // FSocket * sock = nullptr;
  _sock = FTcpSocketBuilder(TEXT("test ros tcp"))
    .AsReusable().AsNonBlocking();

  // _sock->SetReceiveBufferSize(4000000, NewSize); // TODO what is the default?
  _sock->Connect(*addr);
  
  // TODO Wait for successful connection? How? ConnectionState always returns CLOSED. Even after waiting a bit.

  // // Setting up the receiver thread
  std::cout << "Setting up receiver thread..." << std::endl;
  receiverThread = std::move(std::thread([=]() {ReceiverThreadFunction(); return 1; }));
  receiverThreadSetUp = true;

  return true;
}

bool TCPConnection::SendMessage(std::string data){
  // std::string msg = "{\"args\":{\"a\":1,\"b\":2},\"id\":\"call_service:/add_two_ints:23\",\"op\":\"call_service\",\"service\":\"/add_two_ints\"}";
  const uint8 *byte_msg = reinterpret_cast<const uint8*>(data.c_str());
  int32 bytes_sent = 0;
  // TODO check proper casting

  // TODO check errors on send
  _sock->Send(byte_msg, data.length(), bytes_sent);
  std::cout << "Send data: " << data << std::endl;

  return true;
}



bool TCPConnection::SendMessage(const uint8_t *data, unsigned int length){
  // std::cerr << "SendMessage for binary not implemented" << std::endl;
  int32 bytes_sent = 0;
  unsigned int total_bytes_to_send = length;
  // TODO what happens when not all data can be sent in one call?
  
  // Simple checksum
  uint16_t checksum = Fletcher16(data, length);
  
  // Use a timeout if necessary
  FDateTime GraceTimeStart;
  bool GraceTimeActive = false;
  
  while(total_bytes_to_send > 0){
    bool SendResult = _sock->Send(data, total_bytes_to_send, bytes_sent);

    if(bytes_sent == -1){
        std::cout << "#";
        if(!GraceTimeActive){
            std::cerr << "Send returned -1 bytes. Grace Time started" << std::endl; 
            GraceTimeStart = FDateTime::UtcNow();
            GraceTimeActive = true;
        }else{
            // Check elapsed time in Grace Period
            uint8 ElapsedSeconds = FDateTime::UtcNow().ToUnixTimestamp() - GraceTimeStart.ToUnixTimestamp();
            if(ElapsedSeconds >= 4){
                // Cancel this send process
                std::cerr << "Grace Time passed for sending packet. Discarding it." << std::endl; 
                return false;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
        // TODO check for send timeout
    }else{
        GraceTimeActive = false;
    }
    
    std::cout << "Send " << bytes_sent << " bytes from (" << total_bytes_to_send << "/" << length <<") Bytes ";
    if(SendResult){
        std::cout << " T ";
    }else{
        std::cout << " F ";        
    }
    
    total_bytes_to_send -= bytes_sent;
  }
  
  
  // for (int i = 0; i < bytes_sent; i++) {
  //   std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << (int)( data[i] );
  // }
  std::cout << std::dec << "<SendDataEnd:" << checksum << ">" << std::endl;
  return true;
}

uint16_t TCPConnection::Fletcher16( const uint8_t *data, int count ){
     uint16_t sum1 = 0;
     uint16_t sum2 = 0;
     int index;
  
     for( index = 0; index < count; ++index )
     {
        sum1 = (sum1 + data[index]) % 255;
       sum2 = (sum2 + sum1) % 255;
    }
 
    return (sum2 << 8) | sum1;
}

int TCPConnection::ReceiverThreadFunction(){

  std::cout<<"Receiving\n";
  // uint32_t buf_size=1000000; // 1 MB
  // char recv_buffer[buf_size];
  // char* recv_buffer = new char[buf_size];
  // char* recv_buffer = new char[buf_size];
  // std::unique_ptr<char[]> recv_buffer(new char[buf_size]); 

  // Register message callback
  // std::function<void(const json)> message_cb = messageCallback;

  // TODO handle joined messages while reading the buffer
  uint32 count;
  TArray<uint8> binary_buffer;
  uint32_t buffer_size = 10 * 1024 * 1024;
  binary_buffer.Reserve(buffer_size);
  bool bson_state_read_length = true; // indicate that the receiver shall only get 4 bytes to start with
  int32_t bson_msg_length = 0;

  while(!terminateReceiverThread){
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    // std::cout << "!";
    // std::cout.flush();

    count = 0;
    // std::string received_data;
    FString result;

    // TODO
    // How can we check if the connection to the server has been interrupted?
    // Checking the connection state alone doesn't catch simple interruptions 
    // like a crashed server etc.
    //

    if(_sock->GetConnectionState()!=ESocketConnectionState::SCS_Connected){
      std::cout << "Error on connection" << std::endl;
      ReportError(rosbridge2cpp::TransportError::R2C_SOCKET_ERROR);
      return 2; // error while receiving from socket
    }else{
      // std::cout << "c";
      // std::cout.flush();
    }



    int32 bytes_read = 0;
    // std::cout << "o";
    std::cout.flush();
    if(bson_only_mode_){
      // state == read_length
      // if data present:
      //   if state == read length:
      //     buffer.Empty()
      //     Recv 4 bytes and place in buffer
      //     Convert to message length
      //     state == read_message
      //   else:
      //     Recv message_length bytes
      //     append_data_to_buf
      //     if len(buf) == message_length :
      //       state = read_length
      //       callback(buffer)
      //       buffer.Empty()

      // Temporary buffer that receives
      // the different parts of the messages
      // before joining them

      // std::cout << ",";
      // binary_data.Empty();
      if(_sock->HasPendingData(count) && count > 0){
        TArray<uint8> binary_temp;
        std::cout << "d" << std::dec;
        std::cout << count;
        std::cout.flush();

        if(bson_state_read_length){
          std::cout << "Reading length..." << std::endl;
          binary_buffer.Empty();
          binary_temp.SetNumUninitialized(4);
          if( _sock->Recv(binary_temp.GetData(), 4, bytes_read) ){
            if(bytes_read == 4){
              // TODO endian awareness
              // int32_t msg_length = 0;
              bson_msg_length = ( 
                  binary_temp.GetData()[3] << 24 |
                  binary_temp.GetData()[2] << 16 | 
                  binary_temp.GetData()[1] << 8  |
                  binary_temp.GetData()[0]
                  );
              std::cout << std::dec << "Total message length is: " << bson_msg_length << std::endl;
              binary_buffer += binary_temp;
              // Indicate the message retrieval mode
              bson_state_read_length = false;
            }else{
              std::cerr << "bytes_read is not 4 in bson_state_read_length==true. It's: " << bytes_read << std::endl;
            }

          }else{
            std::cerr << "Failed to recv() even though data is pending. Count vs. bytes_read:" << count << "," << bytes_read << std::endl;
          }
        }else{
          std::cout << "Message read mode" << std::endl;
          // Message retreival mode
          //     Recv message_length bytes
          //     append_data_to_buf
          //     if len(buf) == message_length :
          //       state = read_length
          //       callback(buffer)
          //       buffer.Empty()
          binary_temp.SetNumUninitialized(bson_msg_length - 4);
          if( _sock->Recv(binary_temp.GetData(), bson_msg_length - 4, bytes_read) ){
            if(bytes_read > bson_msg_length -4){
              std::cerr << "Read more than bson length -4 !" << std::endl;
            }

            binary_buffer += binary_temp;
            int32 msg_size_in_buffer = 0;
            msg_size_in_buffer = binary_buffer.Num();
            if(msg_size_in_buffer == bson_msg_length){
              bson_state_read_length = true;
              // Full received message!
              for (int i = 0; i < binary_buffer.Num(); i++) {
                std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << (int)( binary_buffer.GetData()[i] );
              }
              std::cout << std::dec;
              bson_t b;
              if(!bson_init_static (&b, binary_buffer.GetData(), msg_size_in_buffer)){
                std::cout << "Error on BSON parse - Ignoring message" << std::endl;
                continue;
              }
              if(incoming_message_callback_bson_){
                incoming_message_callback_bson_(b);
              }

              binary_buffer.Empty();
            }else{
              std::cout << "Binary buffer num is:" << binary_buffer.Num() << std::endl;
            }
          }else{
            std::cerr << "Failed to recv() in message retreival mode even though data is pending. Count vs. bytes_read:" << count << "," << bytes_read << std::endl;
          }
        }
      }
      if(count == 0){
        std::cout.flush();
      }
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }else{
      std::cout << ";";
      std::cout.flush();
      while(_sock->HasPendingData(count) && count > 0){
        FArrayReader data;
        data.SetNumUninitialized(count);

        int32 bytes_read = 0;
        // read pending data into the Data array reader
        if( _sock->Recv(data.GetData(), data.Num(), bytes_read) )
        {
          int32 dest_len = TStringConvert<ANSICHAR,TCHAR>::ConvertedLength((char*)(data.GetData()),data.Num());
          UE_LOG(LogTemp, Verbose, TEXT("count is %d"), count);
          UE_LOG(LogTemp, Verbose, TEXT("bytes_read is %d"), bytes_read);
          UE_LOG(LogTemp, Verbose, TEXT("dest_len will be %i"), dest_len);
          TCHAR* dest = new TCHAR[dest_len+1];
          TStringConvert<ANSICHAR,TCHAR>::Convert(dest, dest_len, (char*)(data.GetData()), data.Num());
          dest[dest_len]='\0';

          result += dest;
          delete dest;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      std::string received_data(TCHAR_TO_UTF8(*result));
      if(received_data.length()==0){
        continue;
      }


      std::cout << "[TCPConnection] Received message(" << received_data.length() << "): " << received_data << std::endl;
      // TODO catch parse error properly
      // auto j = json::parse(received_data);
      json j;
      j.Parse(received_data);

      // TODO Use a thread for the message callback?
      if(_incoming_message_callback)
        _incoming_message_callback(j);

    }
  }

  // Everything went OK - terminateReceiverThread is now true
  return 0;
}


void TCPConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun){
  _incoming_message_callback = fun;
  _callback_function_defined = true;
}

void TCPConnection::RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun){
  incoming_message_callback_bson_ = fun;
  _callback_function_defined = true;
}

void TCPConnection::RegisterErrorCallback(std::function<void(rosbridge2cpp::TransportError)> fun){
  _error_callback = fun;
}
void TCPConnection::ReportError(rosbridge2cpp::TransportError err){
  if(_error_callback)
    _error_callback(err);
}


void TCPConnection::SetTransportMode(rosbridge2cpp::ITransportLayer::TransportMode mode){
  switch(mode){
    case rosbridge2cpp::ITransportLayer::JSON:
      bson_only_mode_ = false;
      break;
    case rosbridge2cpp::ITransportLayer::BSON:
      bson_only_mode_ = true;
      break;
    default:
      std::cerr << "Given TransportMode Not implemented " << std::endl;
  }

}
