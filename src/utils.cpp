#include "utils.hpp"

double millisecondDiff(struct timespec start_time, struct timespec end_time) {
    long diffInNanos = (end_time.tv_sec - start_time.tv_sec) * (long)1e9 + (end_time.tv_nsec - start_time.tv_nsec);
    return (double)diffInNanos / 1000000.0;
}