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
	memset(&msg, 0, sizeof(msg));
	memset(&status, 0, sizeof(status));
	memset(&target, 0, sizeof(target));
	memset(&pitchRoll, 0, sizeof(pitchRoll));
	memset(&flow, 0, sizeof(flow));
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
				if (strstr(s.text, "recieved") == 0) {
					printf("status %s\n", s.text);
				}
			}
			if (msg.msgid == MAVLINK_MSG_ID_D3_PitchRoll) {
				mavlink_d3_pitchroll_t l;
				mavlink_msg_d3_pitchroll_decode(&msg, &l);
				pitchRoll = l;
			}
		}
	}
}

uint16_t MavlinkBridge::sendTarget() {
	mavlink_msg_d3_target_encode(57, 57, &msg, &target);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	write(bridge_tty_fd, &buf, len);
	return len;
}

uint16_t MavlinkBridge::sendFlow() {
	mavlink_msg_d3_flow_encode(57, 57, &msg, &flow);
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

mavlink_d3_pitchroll_t MavlinkBridge::getPitchRoll() {
	return pitchRoll;
}

void MavlinkBridge::threadMain() {
	while (running) {
		readFromStream();
		this_thread::sleep_for(chrono::milliseconds(10));
	}
}

void MavlinkBridge::send_target(long timestamp, float x, float y) {
	target.timestamp = timestamp;
	target.x = x;
	target.y = y;
	sendTarget();
}
void MavlinkBridge::send_flow(long timestamp_from, long timestamp_to, float x, float y) {
	flow.timestamp_from = timestamp_from;
	flow.timestamp_to = timestamp_to;
	target.x = x;
	target.y = y;
	sendTarget();
}
} /* namespace d3 */
