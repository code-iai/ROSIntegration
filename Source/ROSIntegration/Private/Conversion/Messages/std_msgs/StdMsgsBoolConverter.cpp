// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Messages/std_msgs/StdMsgsBoolConverter.h"


UStdMsgsBoolConverter::UStdMsgsBoolConverter()
{
	_MessageType = "std_msgs/Bool";
}

bool UStdMsgsBoolConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) {
	bool KeyFound = false;

	bool Data = (float)GetBoolFromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::Bool(Data));
	return true;
}

bool UStdMsgsBoolConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) {
	auto BoolMessage = StaticCastSharedPtr<ROSMessages::std_msgs::Bool>(BaseMsg);
	*message = BCON_NEW(
		"data", BCON_BOOL(BoolMessage->_Data)
	);
	return true;
}