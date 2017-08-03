#pragma once

#include <list>

#include "rapidjson/document.h"

#include "ros_bridge.h"
#include "types.h"
#include "helper.h"
#include "messages/rosbridge_advertise_msg.h"
#include "messages/rosbridge_publish_msg.h"
#include "messages/rosbridge_subscribe_msg.h"
#include "messages/rosbridge_unsubscribe_msg.h"
#include "messages/rosbridge_unadvertise_msg.h"

using json = rapidjson::Document;

namespace rosbridge2cpp{
  class ROSTopic {
    public:
      // TODO: Implement setter of other options
      ROSTopic (ROSBridge &ros, std::string topic_name, std::string message_type) : 
        ros_(ros), topic_name_(topic_name), message_type_(message_type){
        }

      // Subscribes to a ROS Topic and registers a callback function
      // for incoming messages
      // Multiple callback functions for the same topic within the same instance
      // can be registered.
      // Please note, that these callbacks shouldn't be anonymous entities such as lambdas,
      // to allow to them to unregister them with unsubscribe()
      //
      // For every incoming message, the callback function will receive the "msg" 
      // field of the incoming ROSBridge packet for the given topic
      //
      // *WARNING* When using rapidjson transmission, be aware of moving operations 
      // in the topic callbacks.
      // Things like:
      // std::string x = message.msg_json_["data"];
      // in the callbacks will move the data from the 
      // the json result to the local variable
      // When this happens, other callbacks that receive the same message
      // will read 'Null' on msg_json_
      void Subscribe(FunVrROSPublishMsg callback);

      // Unsubscribe from a given topic
      // If multiple callbacks for this topic have been registered,
      // the given callback will be unregistered in the ROSBridge WITHOUT
      // sending 'unsubscribe' to the rosbridge server.
      // If you're passing the last registered callback to this function,
      // it will be unregistered in the ROSBridge instance
      // AND 'unsubscribe' will be send to the server
      void Unsubscribe(FunVrROSPublishMsg callback);

      // Advertise as a publisher for this topic
      void Advertise();

      // Unadvertise as a publisher for this topic
      void Unadvertise();

      // Publish a message over this topic.
      // If advertise() has not been called before, this will be done in this method beforehand.
      // Please make sure that the message matches the type of the topic,
      // since this will NOT be valided before sending it to the rosbridge.
      //
      // @deprecated, use void Publish(rapidjson::Value &message);
      // void Publish(json &message);

      // Publish a message over this topic.
      // If advertise() has not been called before, this will be done in this method beforehand.
      // Please make sure that the message matches the type of the topic,
      // since this will NOT be valided before sending it to the rosbridge.
      void Publish(rapidjson::Value &message);
      void Publish(bson_t *message);

      std::string GeneratePublishID();

      std::string TopicName(){
        return topic_name_;
      }

    private:
      ROSBridge &ros_;
      std::string topic_name_;
      std::string message_type_;

      // Optional parameters and it's defaults
      bool is_advertised_ = false;
      std::string compression_ = "none";
      int throttle_rate_ = 0;
      bool latch_ = false;
      int queue_size_ = 100;
      int queue_length_ = 0;

      // Householding variables
      std::string advertise_id_ = "";
      std::string subscribe_id_ = "";

      // Count how many callbacks are currently registered in the ROSBridge instance
      int subscription_counter_ = 0;

      // std::list<FunVcrJSON> _callbacks;
  };
}
