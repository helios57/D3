/*
 * timerutil.h
 */

#ifndef TIMERUTIL_H_
#define TIMERUTIL_H_

#include <time.h>

struct timespec timer_start();
long timer_end(struct timespec start_time);

#endif /* TIMERUTIL_H_ */
