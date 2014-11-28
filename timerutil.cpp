/*
 * timerutil.cpp
 *
 *  Created on: Jul 12, 2014
 *      Author: helios
 */

#include "timerutil.h"

// call this function to start a nanosecond-resolution timer
struct timespec timer_start() {
	struct timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

// call this function to end a timer, returning milliseconds elapsed as a long
long int timer_end(struct timespec start_time) {
	struct timespec end_time;
	clock_gettime(CLOCK_REALTIME, &end_time);
	long int diffInMillis = (end_time.tv_nsec / 1000000) - (start_time.tv_nsec / 1000000);
	return diffInMillis;
}
