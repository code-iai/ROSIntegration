#include "ros_topic.h"

namespace rosbridge2cpp{
  void ROSTopic::Subscribe(FunVrROSPublishMsg callback){
    ros_.RegisterTopicCallback(topic_name_, callback); // Register callback in ROSBridge
    subscription_counter_++;

    // Only send subscribe when this ROSTopic hasn't sent this command before
    if(subscribe_id_!="")
      return;

    subscribe_id_ = "";
    subscribe_id_.append("subscribe:");
    subscribe_id_.append(topic_name_);
    subscribe_id_.append(":");
    subscribe_id_.append(std::to_string(++ros_.id_counter));

    ROSBridgeSubscribeMsg cmd(true);
    cmd.id_ = subscribe_id_;
    cmd.topic_ = topic_name_;
    cmd.type_ = message_type_;
    cmd.compression_ = compression_;
    cmd.throttle_rate_ = throttle_rate_;
    cmd.queue_length_ = queue_length_;

    ros_.SendMessage(cmd);
  }

  void ROSTopic::Unsubscribe(FunVrROSPublishMsg callback){
    // We've no active subscription
    if(subscribe_id_ == "") 
      return;

    if(!ros_.UnregisterTopicCallback(topic_name_, callback)){ // Unregister callback in ROSBridge
      // failed to unregister callback - maybe the method is different from already registered callbacks
      std::cerr << "[ROSTopic] Passed unknown callback to ROSTopic::unsubscribe. This callback is not registered in the ROSBridge instance. Aborting..." << std::endl;
      return;
    }

    subscription_counter_--;

    if(subscription_counter_ > 0)
      return;

    std::cout << "[ROSTopic] No callbacks registered anymore - unsubscribe from topic" << std::endl;
    // Handle unsubscription when no callback is registered anymore
    // rapidjson::Document cmd;
    // cmd.SetObject();

    ROSBridgeUnsubscribeMsg cmd(true);
    cmd.id_ = subscribe_id_;
    cmd.topic_ =  topic_name_;

    ros_.SendMessage(cmd);

    subscribe_id_ = "";
    subscription_counter_ = 0; // shouldn't be necessary ...
  }

  void ROSTopic::Advertise(){
    if(is_advertised_) 
      return;

    advertise_id_ = "";
    advertise_id_.append("advertise:");
    advertise_id_.append(topic_name_);
    advertise_id_.append(":");
    advertise_id_.append(std::to_string(++ros_.id_counter));

    ROSBridgeAdvertiseMsg cmd(true);
    cmd.id_ = advertise_id_;
    cmd.topic_ =  topic_name_;
    cmd.type_ =  message_type_;
    cmd.latch_ =  latch_;
    cmd.queue_size_ =  queue_size_;

    ros_.SendMessage(cmd);

    is_advertised_ = true;
  }
  void ROSTopic::Unadvertise(){
    if(!is_advertised_)
      return;

    ROSBridgeUnadvertiseMsg cmd(true);
    cmd.id_ = advertise_id_;
    cmd.topic_ =  topic_name_;

    ros_.SendMessage(cmd);

    is_advertised_ = false;
  }
  // void ROSTopic::Publish(json &message){
  //   if(!is_advertised_)
  //     Advertise();

  //   std::string publish_id;
  //   publish_id.append("publish:");
  //   publish_id.append(topic_name_);
  //   publish_id.append(":");
  //   publish_id.append(std::to_string(++ros_.id_counter));

  //   rapidjson::Document cmd;
  //   cmd.SetObject();
  //   cmd.AddMember("op","publish", cmd.GetAllocator());
  //   cmd.AddMember("id", publish_id, cmd.GetAllocator());
  //   cmd.AddMember("topic", topic_name_, cmd.GetAllocator());
  //   cmd.AddMember("msg", message, cmd.GetAllocator());
  //   cmd.AddMember("latch", latch_, cmd.GetAllocator());

  //   std::cout << "[ROSTopic] Publishing data " << Helper::get_string_from_rapidjson(cmd);


  //   ros_.SendMessage(cmd);
  // }

  void ROSTopic::Publish(rapidjson::Value &message){
    if(!is_advertised_)
      Advertise();

    std::string publish_id = GeneratePublishID();

    ROSBridgePublishMsg cmd(true);
    cmd.id_ =  publish_id;
    cmd.topic_ =  topic_name_;
    cmd.msg_json_ =  message;
    cmd.latch_ =  latch_;

    ros_.SendMessage(cmd);
  }

  void ROSTopic::Publish(bson_t *message){
    if(!is_advertised_)
      Advertise();

    assert(message);

    std::string publish_id = GeneratePublishID();

    ROSBridgePublishMsg cmd(true);
    cmd.id_ =  publish_id;
    cmd.topic_ =  topic_name_;
    cmd.msg_bson_ =  message;
    cmd.latch_ =  latch_;

    ros_.SendMessage(cmd);
  }

  std::string ROSTopic::GeneratePublishID(){
    std::string publish_id;
    publish_id.append("publish:");
    publish_id.append(topic_name_);
    publish_id.append(":");
    publish_id.append(std::to_string(++ros_.id_counter));
    return publish_id;
  }
}
