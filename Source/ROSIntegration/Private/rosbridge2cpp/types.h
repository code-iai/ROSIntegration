#pragma once
#include <functional>

#include "rapidjson/document.h"

#include "messages/rosbridge_msg.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_publish_msg.h"
#include "messages/rosbridge_service_response_msg.h"

namespace rosbridge2cpp{ 
  using json = rapidjson::Document;
  typedef std::function<void(const json&)> FunVcrJSON;
  // typedef std::function<void(ROSBridgeMsg&)> FunVrROSMSG;
  typedef std::function<void(const ROSBridgePublishMsg&)> FunVrROSPublishMsg;
  typedef std::function<void(ROSBridgeServiceResponseMsg&)> FunVrROSServiceResponseMsg;
  typedef std::function<void(ROSBridgeCallServiceMsg&, ROSBridgeServiceResponseMsg&, rapidjson::Document::AllocatorType&)> FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator;
  typedef std::function<void(ROSBridgeCallServiceMsg&, ROSBridgeServiceResponseMsg&)> FunVrROSCallServiceMsgrROSServiceResponseMsg;
  // typedef std::function<json(json&)> FunJSONcrJSON;

  enum class TransportError{ R2C_SOCKET_ERROR, R2C_CONNECTION_CLOSED };
}
