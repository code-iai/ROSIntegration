#pragma once

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"

namespace ROSMessages {
	namespace sensor_msgs {
		class CompressedImage : public FROSBaseMsg {
		public:
			CompressedImage() {
				_MessageType = "sensor_msgs/CompressedImage";
			}

			/** Header timestamp should be acquisition time of image
			 *  Header frame_id should be optical frame of camera
			 *  origin of frame should be optical center of camera
			 *  +x should point to the right in the image
			 *  +y should point down in the image
			 *  +z should point into to plane of the image
			 */
			ROSMessages::std_msgs::Header header;

			/** Specifies the format of the data
			 *  Acceptable values: jpeg, png
			 */
			FString format;

			/** Compressed image buffer */
			const uint8* data;

			/** image buffer size */
			int data_size;
		};
	}
}
