#include <stdio.h>
#include <stdint.h>
#include <time.h>

#ifndef VISION_UTILS_H
double millisecondDiff(struct timespec start_time, struct timespec end_time);
uint64_t CurrentTime_nanoseconds();
#define VISION_UTILS_H
#endif