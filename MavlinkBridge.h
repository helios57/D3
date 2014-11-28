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
#include "mavlink/pixhawk/mavlink.h"
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
		mavlink_d3_target_t target;
		mavlink_d3_pitchroll_t pitchRoll;
		mavlink_d3_flow_t flow;
		struct termios bridge_tio;
		bool running;
		bool connected;
		int bridge_tty_fd;
		unsigned char bridge_c;
		thread *receiverThread;

		void close();
		void initStreams();
		void readFromStream();
		uint16_t sendTarget();
		uint16_t sendFlow();
		void threadMain();

	public:
		MavlinkBridge();
		virtual ~MavlinkBridge();
		void start();
		void stop();
		mavlink_d3_pitchroll_t getPitchRoll();
		void send_target(long timestamp, float x, float y);
		void send_flow(long timestamp_from, long timestamp_to, float x, float y);
};

} /* namespace d3 */

#endif /* MAVLINKBRIDGE_H_ */
