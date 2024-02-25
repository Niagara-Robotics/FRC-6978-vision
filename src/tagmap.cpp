#include <stdlib.h>
#include <map>
#include <vector>
#include <math.h>

#include <opencv2/core.hpp>

#include <tagmap.h>

double toRadians(double degrees) {
    return degrees / 180.0 * M_PI;
}

void add_tag(int id, double x, double y, double z, double angle, std::map<int, std::vector<cv::Point3d>> *tag_map) {
    angle += M_PI / 2.0; //field tags are specified in heading rather than transform angle

    x *= CONVERSION_FACTOR;
    y *= CONVERSION_FACTOR;
    z *= CONVERSION_FACTOR;

    std::vector<cv::Point3d> points;
    points.push_back(cv::Point3d(x-(TAG_WIDTH*cos(angle)),   y-(TAG_WIDTH*sin(angle)),      z)); //bottom left
    points.push_back(cv::Point3d(x+(TAG_WIDTH*cos(angle)),   y+(TAG_WIDTH*sin(angle)),      z)); //bottom right
    points.push_back(cv::Point3d(x+(TAG_WIDTH*cos(angle)),   y+(TAG_WIDTH*sin(angle)),      z+TAG_WIDTH)); //top right
    points.push_back(cv::Point3d(x-(TAG_WIDTH*cos(angle)),   y-(TAG_WIDTH*sin(angle)),      z+TAG_WIDTH)); //top left
    tag_map->emplace(id, points);
}

std::map<int, std::vector<cv::Point3d>> build_map() {
    std::map<int, std::vector<cv::Point3d>> tag_map;
    add_tag(1,  593.68,   9.68, 53.38, toRadians(120.0), &tag_map);
    add_tag(2,  637.21,  34.79, 53.38, toRadians(120.0), &tag_map);
    add_tag(3,  652.73, 196.17, 57.13, toRadians(180.0), &tag_map);
    add_tag(4,  652.73, 218.42, 57.13, toRadians(180.0), &tag_map);
    add_tag(5,  578.77, 323.00, 53.38, toRadians(270.0), &tag_map);
    add_tag(6,  072.50, 323.00, 53.38, toRadians(270.0), &tag_map);
    add_tag(7, -001.50, 218.42, 57.13, toRadians(  0.0), &tag_map);
    add_tag(8, -001.50, 196.17, 57.13, toRadians(  0.0), &tag_map);
    add_tag(9,  014.02, 034.79, 53.38, toRadians( 60.0), &tag_map);
    add_tag(10, 057.54, 009.68, 53.38, toRadians( 60.0), &tag_map);
    add_tag(11, 468.69, 146.19, 52.00, toRadians(300.0), &tag_map);
    add_tag(12, 468.69, 177.10, 52.00, toRadians( 60.0), &tag_map);
    add_tag(13, 441.74, 161.62, 52.00, toRadians(180.0), &tag_map);
    add_tag(14, 209.48, 161.42, 52.00, toRadians(  0.0), &tag_map);
    add_tag(15, 182.73, 177.10, 52.00, toRadians(120.0), &tag_map);
    add_tag(16, 182.73, 146.19, 52.00, toRadians(240.0), &tag_map);
    return tag_map;
}

std::map<int, std::vector<cv::Point3d>> build_practice_map() {
    double tagw = 165.0;
    std::map<int, std::vector<cv::Point3d>> tag_map;
    
    tag_map[2].push_back(cv::Point3d(0,0,1355));
    tag_map[2].push_back(cv::Point3d(0+tagw,0,1355));
    tag_map[2].push_back(cv::Point3d(0+tagw,0,1355+tagw));
    tag_map[2].push_back(cv::Point3d(0,0,1355+tagw));
    
    tag_map[3].push_back(cv::Point3d(397.5,0,1355)); //397.5, 1355
    tag_map[3].push_back(cv::Point3d(397.5+tagw,0,1355));
    tag_map[3].push_back(cv::Point3d(397.5+tagw,0,1355+tagw));
    tag_map[3].push_back(cv::Point3d(397.5,0,1355+tagw));
    return tag_map;
}

std::map<int, std::vector<cv::Point3d>> build_home_map() {
    std::map<int, std::vector<cv::Point3d>> tag_map;
    add_tag(2, 0, 0, 53.4, 0, &tag_map);
    add_tag(3, 25.062, 0, 53.4, 0, &tag_map);
    return tag_map;
}