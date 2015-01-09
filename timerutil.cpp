/*
 * timerutil.cpp
 */

#include "timerutil.h"

/// call this function to start a nanosecond-resolution timer
struct timespec timer_start() {
	struct timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

/// call this function to end a timer, returning microseconds elapsed as a long
long int timer_end(struct timespec start_time) {
	struct timespec end_time;
	clock_gettime(CLOCK_REALTIME, &end_time);
	long int diffInUs = (end_time.tv_sec - start_time.tv_sec) * 1000000 + (end_time.tv_nsec - start_time.tv_nsec) / 1000;
	return diffInUs;
}
