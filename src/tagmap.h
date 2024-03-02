
#ifndef TAGMAP_H

#define TAG_WIDTH 82.55 //millimeters
#define CONVERSION_FACTOR 25.4

std::map<int, std::vector<cv::Point3d>> build_map();
std::map<int, std::vector<cv::Point3d>> build_practice_map();
std::map<int, std::vector<cv::Point3d>> build_home_map();

#define TAGMAP_H
#endif