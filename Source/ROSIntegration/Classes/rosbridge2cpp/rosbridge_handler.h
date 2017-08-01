#pragma once
#include <string>

class ROSBridgeHandler{
  public:
  ROSBridgeHandler() : _TestString("foobar"){
  }

  void publish();

  std::string _TestString;
};