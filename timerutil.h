/*
 * timerutil.h
 *
 *  Created on: Jul 12, 2014
 *      Author: helios
 */

#ifndef TIMERUTIL_H_
#define TIMERUTIL_H_

#include <time.h>

struct timespec timer_start();
long timer_end(struct timespec start_time);

#endif /* TIMERUTIL_H_ */
