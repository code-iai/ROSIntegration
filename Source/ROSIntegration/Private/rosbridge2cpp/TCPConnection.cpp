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
  //_sock = FTcpSocketBuilder(TEXT("test ros tcp"))
  //  .AsReusable().AsNonBlocking();

  _sock = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("test ros tcp"), false);

  // _sock->SetReceiveBufferSize(4000000, NewSize); // TODO what is the default?

  if (!_sock->Connect(*addr))
	  return false;

  // // Setting up the receiver thread
  std::cout << "Setting up receiver thread..." << std::endl;
  //receiverThread = std::move(std::thread([=]() {ReceiverThreadFunction(); return 1; }));

  run_receiver_thread = true;
  receiverThread = std::thread(&TCPConnection::ReceiverThreadFunction, this);
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
 
  // Simple checksum
  //uint16_t checksum = Fletcher16(data, length);
  
  int32 bytes_sent = 0;
  unsigned int total_bytes_to_send = length;
  int32 num_tries = 0;
  while(total_bytes_to_send > 0 && num_tries < 3)
  {
    bool SendResult = _sock->Send(data, total_bytes_to_send, bytes_sent);

    if(SendResult)
    {
        data += bytes_sent;
    }
    else
    {
        num_tries++;
    }
    
    total_bytes_to_send -= bytes_sent;
  }

  return total_bytes_to_send == 0;
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

  // TODO handle joined messages while reading the buffer
  uint32 count;
  TArray<uint8> binary_buffer;
  uint32_t buffer_size = 10 * 1024 * 1024;
  binary_buffer.Reserve(buffer_size);
  bool bson_state_read_length = true; // indicate that the receiver shall only get 4 bytes to start with
  int32_t bson_msg_length = 0;
  int return_value = 0;

  while(run_receiver_thread){

    ESocketConnectionState ConnectionState = _sock->GetConnectionState();
    if( ConnectionState != ESocketConnectionState::SCS_Connected ){
        if (ConnectionState == SCS_NotConnected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            std::cout << "Error on connection" << std::endl;
            ReportError(rosbridge2cpp::TransportError::R2C_SOCKET_ERROR);
            run_receiver_thread = false;
            return_value = 2; // error while receiving from socket
            continue;
        }
    }

    count = 0;
    if (!_sock->HasPendingData(count) || count == 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
    }

    if(bson_only_mode_){
        TArray<uint8> binary_temp;
        if(bson_state_read_length){
            binary_buffer.Empty();
            binary_temp.SetNumUninitialized(4);
            int32 bytes_read = 0;
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
            // Message retreival mode
            //     Recv message_length bytes
            //     append_data_to_buf
            //     if len(buf) == message_length :
            //       state = read_length
            //       callback(buffer)
            //       buffer.Empty()
            binary_temp.SetNumUninitialized(bson_msg_length - 4);
            int32 bytes_read = 0;
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
                    bson_t b;
                    if(!bson_init_static (&b, binary_buffer.GetData(), msg_size_in_buffer)){
                        std::cout << "Error on BSON parse - Ignoring message" << std::endl;
                        continue;
                    }
                    if(incoming_message_callback_bson_){
                        incoming_message_callback_bson_(b);
                    }

                    binary_buffer.Empty(binary_buffer.Max());
                }else{
                    std::cout << "Binary buffer num is:" << binary_buffer.Num() << std::endl;
                }
            }else{
                std::cerr << "Failed to recv() in message retreival mode even though data is pending. Count vs. bytes_read:" << count << "," << bytes_read << std::endl;
            }
        }
    }else{
      FString result;
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

          delete[] dest;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      std::string received_data(TCHAR_TO_UTF8(*result));
      if(received_data.length()==0){
        continue;
      }

      // TODO catch parse error properly
      // auto j = json::parse(received_data);
      json j;
      j.Parse(received_data);

      if(_incoming_message_callback)
        _incoming_message_callback(j);
    }
  }

  return return_value;
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

bool TCPConnection::IsHealthy() const
{
    return run_receiver_thread;
}
