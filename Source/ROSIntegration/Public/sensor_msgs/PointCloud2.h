#pragma once 

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"

namespace ROSMessages {
	namespace sensor_msgs {
		class PointCloud2 : public FROSBaseMsg {
		public:

			// we use a local PointField definition here instead of sensor_msgs/PointField 
			// to avoid unecessary bloat by deriving from FROSBaseMsg and it is only used for PointCloud2 msg anyway
			struct PointField
			{
				enum EType
				{
					INT8 = 1,
					UINT8 = 2,
					INT16 = 3,
					UINT16 = 4,
					INT32 = 5,
					UINT32 = 6,
					FLOAT32 = 7,
					FLOAT64 = 8
				};

				FString name;
				uint32 offset;
				EType  datatype;
				uint32 count;
			};

			PointCloud2() {
				_MessageType = "sensor_msgs/PointCloud2";
			}

			ROSMessages::std_msgs::Header header;

			uint32 height;
			uint32 width;

			TArray<PointField> fields;

			bool	is_bigendian;
			uint32	point_step;
			uint32	row_step;

			// To avoid copy operations of the point data, hand over a pointer to the data. 
			// Please note, that the memory this pointer points to must be valid until this message has been published.
			// When receiving, please note that ROS sends vectors padded to 16 bytes, with 3 floats + 4 byte padding.
			const uint8* data_ptr;

			bool is_dense;
		};
	}
}
