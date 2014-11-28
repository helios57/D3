/*
 * MavlinkBridge.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: helios
 */

#include "MavlinkBridge.h"
namespace d3 {

void blubber() {

}
MavlinkBridge::MavlinkBridge() :
		running(true), connected(false), bridge_tty_fd(0), bridge_c('D') {
	receiverThread = 0;
	memset(&setpointAdjustment, 0, sizeof(mavlink_set_position_target_local_ned_t));
	memset(&attitude, 0, sizeof(mavlink_attitude_t));
	initStreams();
}
MavlinkBridge::~MavlinkBridge() {
}
void MavlinkBridge::close() {
	::close(bridge_tty_fd);
	bridge_tty_fd = 0;
	connected = false;
}
void MavlinkBridge::initStreams() {
	memset(&bridge_tio, 0, sizeof(bridge_tio));
	bridge_tio.c_iflag = 0;
	bridge_tio.c_oflag = 0;
	bridge_tio.c_cflag = CS8 | CREAD | CLOCAL;
	bridge_tio.c_lflag = 0;
	bridge_tio.c_cc[VMIN] = 1;
	bridge_tio.c_cc[VTIME] = 5;
	bridge_tty_fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);

	if (bridge_tty_fd <= 0) {
		printf("open(\"/dev/ttyACM0\", O_RDWR | O_NONBLOCK)==0");
		close();
		return;
	}
	if (cfsetospeed(&bridge_tio, B57600) != 0) { // 115200 baud = B115200
		printf("cfsetospeed(&tio, B115200)!= 0");
		close();
		return;
	}
	if (cfsetispeed(&bridge_tio, B57600) != 0) { // 115200 baud = B115200
		printf("cfsetispeed(&tio, B115200)!= 0");
		close();
		return;
	}
	if (tcsetattr(bridge_tty_fd, TCSANOW, &bridge_tio) != 0) {
		printf("tcsetattr(tty_fd, TCSANOW, &tio)!= 0");
		close();
		return;
	}
	char cmd[] = "sh /etc/init.d/rc.usb\n";
	sleep(2);
	while (read(bridge_tty_fd, &bridge_c, 1) > 0 && bridge_c != 0) {
		write(STDOUT_FILENO, &bridge_c, 1); // if new data is available on the serial port, print it out
	}
	write(bridge_tty_fd, cmd, sizeof(cmd));
	sleep(2);
	while (read(bridge_tty_fd, &bridge_c, 1) > 0 && bridge_c != 0) {
		write(STDOUT_FILENO, &bridge_c, 1); // if new data is available on the serial port, print it out
	}
	connected = true;
}

void MavlinkBridge::readFromStream() {
	int counter = 0;
	while (running && connected && read(bridge_tty_fd, &bridge_c, 1) > 0 && counter++ < 1000) {
		if (mavlink_parse_char(MAVLINK_COMM_0, bridge_c, &msg, &status)) {
			if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
				mavlink_statustext_t s;
				mavlink_msg_statustext_decode(&msg, &s);
				if (strstr(s.text, "recieved") == 0){
					printf("status %s\n", s.text);
				}
			}
			if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
				mavlink_attitude_t at;
				mavlink_msg_attitude_decode(&msg, &at);
				attitude = at;
			}
			if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
				mavlink_local_position_ned_t localPosition_t;
				mavlink_msg_local_position_ned_decode(&msg, &localPosition_t);
				localPosition = localPosition_t;
			}
			if (msg.msgid == MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED) {
				mavlink_position_target_local_ned_t localPositionTarget_t;
				mavlink_msg_position_target_local_ned_decode(&msg, &localPositionTarget_t);
				localPositionTarget = localPositionTarget_t;
			}
		}
	}
}

uint16_t MavlinkBridge::sendMessage() {
	mavlink_msg_set_position_target_local_ned_encode(57, 57, &msg, &setpointAdjustment);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	write(bridge_tty_fd, &buf, len);
	return len;
}

void MavlinkBridge::start() {
	receiverThread = new thread(&MavlinkBridge::threadMain, this);
}

void MavlinkBridge::stop() {
	running = false;
	delete (receiverThread);
}

mavlink_attitude_t MavlinkBridge::getAttitude() {
	return attitude;
}

void MavlinkBridge::threadMain() {
	while (running) {
		readFromStream();
		this_thread::sleep_for(chrono::milliseconds(10));
	}
}

mavlink_local_position_ned_t MavlinkBridge::getLocalPostition() {
	return localPosition;
}

mavlink_position_target_local_ned_t MavlinkBridge::getLocalPositionTarget() {
	return localPositionTarget;
}

void MavlinkBridge::sendCorrection(float x, float y, float z) {
	setpointAdjustment.x = x;
	setpointAdjustment.y = y;
	setpointAdjustment.z = z;
	sendMessage();
}
} /* namespace d3 */
