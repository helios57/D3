/*
 * MavlinkBridge.h
 *
 *  Created on: Oct 24, 2014
 *      Author: helios
 */

#ifndef MAVLINKBRIDGE_H_
#define MAVLINKBRIDGE_H_

#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include "mavlink/common/mavlink.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <thread>

namespace d3 {
using namespace std;

class MavlinkBridge {
	private:
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		mavlink_message_t msg;
		mavlink_status_t status;
		mavlink_set_position_target_local_ned_t setpointAdjustment;
		mavlink_attitude_t attitude;
		mavlink_local_position_ned_t localPosition;
		mavlink_position_target_local_ned_t localPositionTarget;
		struct termios bridge_tio;
		bool running;
		bool connected;
		int bridge_tty_fd;
		unsigned char bridge_c;
		thread *receiverThread;

		void close();
		void initStreams();
		void readFromStream();
		uint16_t sendMessage();
		void threadMain();

	public:
		MavlinkBridge();
		virtual ~MavlinkBridge();
		void start();
		void stop();
		mavlink_attitude_t getAttitude();
		mavlink_local_position_ned_t getLocalPostition();
		mavlink_position_target_local_ned_t getLocalPositionTarget();
		void sendCorrection(float x, float y, float z);

};

} /* namespace d3 */

#endif /* MAVLINKBRIDGE_H_ */
