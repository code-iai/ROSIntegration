#include "Conversion/Services/std_srvs/StdSrvsTriggerResponseConverter.h"

#include "std_srvs/TriggerResponse.h"
#include <bson.h>


UStdSrvsTriggerResponseConverter::UStdSrvsTriggerResponseConverter()
{
	_ServiceType = "std_srvs/Trigger";
}

bool UStdSrvsTriggerResponseConverter::ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response)
{
	bool key_found = false;
	*Response = MakeShareable(new std_srvs::FTriggerResponse);
	auto ServiceResponse = StaticCastSharedPtr<std_srvs::FTriggerResponse>(*Response);

	ServiceResponse->_Result = res.result_;

	ServiceResponse->_success = rosbridge2cpp::Helper::get_bool_by_key("values.success", *res.full_msg_bson_, key_found);
	if (!key_found) {
		UE_LOG(LogTemp, Error, TEXT("Key values.success not present in data"));
		return false;
	}
	
	std::string messageStd = rosbridge2cpp::Helper::get_utf8_by_key("values.message", *res.full_msg_bson_, key_found);
	ServiceResponse->_message = messageStd.c_str();
	if (!key_found) {
		UE_LOG(LogTemp, Error, TEXT("Key values.message not present in data"));
		return false;
	}

	return true;
}

bool UStdSrvsTriggerResponseConverter::ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res)
{
	auto ServiceResponse = StaticCastSharedPtr<std_srvs::FTriggerResponse>(Response);

	res.result_ = ServiceResponse->_Result;
	BSON_APPEND_BOOL(res.values_bson_, "success", ServiceResponse->_success);
	BSON_APPEND_UTF8(res.values_bson_, "message", TCHAR_TO_UTF8(*ServiceResponse->_message));
	return true;
}

TSharedPtr<FROSBaseServiceResponse> UStdSrvsTriggerResponseConverter::AllocateConcreteResponse()
{
	return MakeShareable(new std_srvs::FTriggerResponse);
}