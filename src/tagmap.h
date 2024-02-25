
#ifndef TAGMAP_H

#define TAG_WIDTH 82.5 //millimeters
#define CONVERSION_FACTOR 25.4

std::map<int, std::vector<cv::Point3d>> build_map();
std::map<int, std::vector<cv::Point3d>> build_practice_map();

#define TAGMAP_H
#endif