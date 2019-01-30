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

            FString format;         // Specifies the format of the data
                                    //   Acceptable values: jpeg, png

            const uint8* data;      // Compressed image buffer

            int data_size;          // image buffer size

        };
    }
}