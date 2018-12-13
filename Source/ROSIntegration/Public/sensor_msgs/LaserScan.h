#pragma once

#include "ROSBaseMsg.h"
#include "std_msgs/Header.h"

namespace ROSMessages {
	namespace sensor_msgs {

		/** Single scan from a planar laser range-finder.
		 *
		 * If you have another ranging device with different behavior(e.g.a sonar
		 * array), please find or create a different message, since applications
		 * will make fairly laser - specific assumptions about this data
		 */
		class LaserScan : public FROSBaseMsg {
		public:
			LaserScan() {
				_MessageType = "sensor_msgs/LaserScan";
			}


			/** timestamp in the header is the acquisition time of the first ray in the scan.
			 *
			 * In frame frame_id, angles are measured around
			 * the positive Z axis(counterclockwise, if Z is up)
			 * with zero angle being forward along the x axis
			 */
			ROSMessages::std_msgs::Header header;

			float angle_min;		// start angle of the scan[rad]
			float angle_max;		// end angle of the scan[rad]
			float angle_increment;	// angular distance between measurements[rad]

			float time_increment;	// time between measurements[seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
			float scan_time;		// time between scans[seconds]

			float range_min;		// minimum range value[m]
			float range_max;		// maximum range value[m]

			TArray<float> ranges;		// range data[m](Note: values < range_min or > range_max should be discarded)
			TArray<float> intensities;	// intensity data[device - specific units]. If your
										// device does not provide intensities, please leave
										// the array empty.
		};
	}
}
