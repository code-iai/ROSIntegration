#pragma once

#include <CoreMinimal.h>
#include "Conversion/Messages/BaseMessageConverter.h"
#include "mavros_msgs/HomePosition.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geographic_msgs/GeoPoint.h"
#include "Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsQuaternionConverter.h"
#include "Conversion/Messages/geometry_msgs/GeometryMsgsVector3Converter.h"
#include "Conversion/Messages/geographic_msgs/GeographicMsgsGeoPointConverter.h"
#include "MavrosMsgsHomePositionConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UMavrosMsgsHomePositionConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UMavrosMsgsHomePositionConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_home_position(bson_t *b, FString key, ROSMessages::mavros_msgs::HomePosition* msg, bool LogOnErrors = true)
	{
		bool KeyFound = true;

		KeyFound &= UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header);
		KeyFound &= UGeometryMsgsPointConverter::_bson_extract_child_point(b, key + ".position", &msg->position);
		KeyFound &= UGeometryMsgsQuaternionConverter::_bson_extract_child_quaternion(b, key + ".orientation", &msg->orientation);
		KeyFound &= UGeometryMsgsVector3Converter::_bson_extract_child_vector3(b, key + ".approach", &msg->approach);
		KeyFound &= UGeographicMsgsGeoPointConverter::_bson_extract_child_geo_point(b, key + ".geo", &msg->geo);

		return KeyFound;
	}

	static void _bson_append_child_home_position(bson_t *b, const char *key, const ROSMessages::mavros_msgs::HomePosition* msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_home_position(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_home_position(bson_t *b, const ROSMessages::mavros_msgs::HomePosition* msg)
	{
		UStdMsgsHeaderConverter::_bson_append_header(b, &(msg->header));
		UGeometryMsgsPointConverter::_bson_append_child_point(b, "position", &(msg->position));
		UGeometryMsgsQuaternionConverter::_bson_append_child_quaternion(b, "orientation", &(msg->orientation));
		UGeometryMsgsVector3Converter::_bson_append_child_vector3(b, "approach", &(msg->approach));
		UGeographicMsgsGeoPointConverter::_bson_append_child_geo_point(b, "geo", &(msg->geo));
	}
};
