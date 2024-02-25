#include "utils.hpp"
#include <stdlib.h>
#include <chrono>

double millisecondDiff(struct timespec start_time, struct timespec end_time) {
    long diffInNanos = (end_time.tv_sec - start_time.tv_sec) * (long)1e9 + (end_time.tv_nsec - start_time.tv_nsec);
    return (double)diffInNanos / 1000000.0;
}

uint64_t CurrentTime_nanoseconds()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>
              (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
} 