// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Messages/std_msgs/StdMsgsEmptyConverter.h"


UStdMsgsEmptyConverter::UStdMsgsEmptyConverter()
{
	_MessageType = "std_msgs/Empty";
}

bool UStdMsgsEmptyConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {


	return true;
}

bool UStdMsgsEmptyConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {

	return true;
}