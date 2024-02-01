#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <filesystem>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include "utils.hpp"
#include "streamer.hpp"

using namespace std;


int bigPipeline(int, char**)
{
    cv::Mat frame, hsv, threshold, eroded;
    cv::VideoCapture cap;

    int deviceID = 3;//
    int apiID = cv::CAP_V4L2;
    cap.open(deviceID, apiID);
    u_int32_t codec = 0x47504A4D;
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_FOURCC, codec);
    cap.set(cv::CAP_PROP_EXPOSURE, 70);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 60);

    if(!cap.isOpened()) {
        cerr << "Unable to open camera\n";
        return -1;
    }

    std::condition_variable cond;
    vision::Streamer smr = vision::Streamer("maincam", 8008, &eroded, &cond);

    struct timespec start_time, end_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);

    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = 4;
    cv::Mat erosion_kernel = cv::getStructuringElement( erosion_type,
                    cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                    cv::Point( erosion_size, erosion_size ) );
    vector<cv::Vec4i> hierarchy;
    vector<vector<cv::Point>> contours;

    printf("x,y,cap,convert,contour,filter");

    for(;;) {
        cap.read(frame);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cap_time);
        //cond.notify_all();

        
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 130, 60), cv::Scalar(21,255,255), eroded);
        //cv::erode(threshold, eroded, erosion_kernel);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &convert_time);

        cv::findContours(eroded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &contour_time);
        int largest = 0;
        int largestIndex = -1;
        int area = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            area = cv::contourArea(contours[i]);
            if(area < 10) continue;
            if(area > largest) {
                largest = area;
                largestIndex = i;
            }

        }
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);
        if(largestIndex >= 0) {
            cv::Rect box = cv::boundingRect(contours[largestIndex]);
            cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0));
            printf("%i,%i,%fms\n", 
                box.x, 
                box.y, 
                millisecondDiff(start_time, end_time)
                //millisecondDiff(cap_time, convert_time),
                //millisecondDiff(convert_time, contour_time),
                //millisecondDiff(contour_time, filter_time)
            );
        }
        
    }
}

int basicCap()
{
    cv::Mat frame, hsv, threshold, eroded;
    cv::VideoCapture cap;

    int deviceID =0;
    int apiID = cv::CAP_V4L2;
    cap.open(deviceID, apiID);
    //u_int32_t codec = 0x47504A4D;
    u_int32_t codec = 'YUYV';
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_FOURCC, codec);
    cap.set(cv::CAP_PROP_EXPOSURE, 70);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 60);

    if(!cap.isOpened()) {
        cerr << "Unable to open camera\n";
        return -1;
    }

    std::condition_variable cond;
    vision::Streamer smr = vision::Streamer("maincam", 8008, &frame, &cond);

    struct timespec start_time, cap_time, end_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);

    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = 4;
    cv::Mat erosion_kernel = cv::getStructuringElement( erosion_type,
                    cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                    cv::Point( erosion_size, erosion_size ) );
    vector<cv::Vec4i> hierarchy;
    vector<vector<cv::Point>> contours;

    printf("x,y,cap,convert,contour,filter");

    for(;;) {
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
        cap.grab();
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cap_time);
        cap.retrieve(frame);
        cond.notify_all();
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);            
        printf("Capture: %fms,%fms\n", 
            millisecondDiff(start_time, cap_time),
            millisecondDiff(cap_time, end_time)
            //millisecondDiff(convert_time, contour_time),
            //millisecondDiff(contour_time, filter_time)
        );
        if(millisecondDiff(start_time, end_time) < 16.66) {
            usleep((int)(1000.0*(16.66 - millisecondDiff(start_time, end_time))));
        }
    }
}

int apriltag(cv::VideoCapture cap)
{
    cv::Mat frame, gray;

    //u_int32_t codec = 0x47504A4D;
    u_int32_t codec = 'YUYV';
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_FOURCC, codec);
    cap.set(cv::CAP_PROP_EXPOSURE, 120);
    cap.set(cv::CAP_PROP_FOCUS, 255);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 60);

    if(!cap.isOpened()) {
        cerr << "Unable to open camera\n";
        return -1;
    }

    std::condition_variable cond;
    vision::Streamer smr = vision::Streamer("maincam", 8008, &frame, &cond);

    struct timespec start_time, cap_time, end_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);

    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = 4;
    cv::Mat erosion_kernel = cv::getStructuringElement( erosion_type,
                    cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                    cv::Point( erosion_size, erosion_size ) );
    vector<cv::Vec4i> hierarchy;
    vector<vector<cv::Point>> contours;

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    printf("x,y,cap,convert,contour,filter");

    float cam[10] = {646.03613227,           0,         296.1850407, 
                    0,            645.91733005,         219.25043667,
                    0,                       0,                    1};
    cv::Mat camera_matrix = cv::Mat(3,3,5, cam);

    float dist[5] = {0.17413211, -0.47767714, -0.00792523, -0.00707075, 0.60882555};
    cv::Mat dist_matrix = cv::Mat(cv::Size(1,5), 5, dist);

    std::vector<cv::Point3d> tag_points = std::vector<cv::Point3d>();

    tag_points.push_back(cv::Point3d(0,0,0));
    tag_points.push_back(cv::Point3d(66,0,0));
    tag_points.push_back(cv::Point3d(66,66,0));
    tag_points.push_back(cv::Point3d(0,66,0));

    std::vector<cv::Point3f> vis_points = std::vector<cv::Point3f>();

    vis_points.push_back(cv::Point3f(0,0,0));
    vis_points.push_back(cv::Point3f(66,0,0));
    vis_points.push_back(cv::Point3f(66,66,0));
    vis_points.push_back(cv::Point3f(0,66,0));

    for(;;) {
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
        cap.grab();
        cap.retrieve(frame);
        
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cap_time);

        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
        
        zarray_t *detections = apriltag_detector_detect(td, &im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                std::vector<cv::Point2d> det_points = std::vector<cv::Point2d>();

                det_points.push_back(cv::Point2d(det->p[0][0], det->p[0][1]));
                det_points.push_back(cv::Point2d(det->p[1][0], det->p[1][1]));
                det_points.push_back(cv::Point2d(det->p[2][0], det->p[2][1]));
                det_points.push_back(cv::Point2d(det->p[3][0], det->p[3][1]));
                cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                        cv::Point(det->p[1][0], det->p[1][1]),
                        cv::Scalar(0, 0xff, 0), 2);
                cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                        cv::Point(det->p[3][0], det->p[3][1]),
                        cv::Scalar(0, 0, 0xff), 2);
                cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                        cv::Point(det->p[2][0], det->p[2][1]),
                        cv::Scalar(0xff, 0, 0), 2);
                cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
                        cv::Point(det->p[3][0], det->p[3][1]),
                        cv::Scalar(0xff, 0, 0), 2);
                cv::Mat rvec, tvec;
                cv::solvePnP(tag_points, det_points, camera_matrix, dist_matrix, rvec, tvec);

                std::vector<cv::Point2f> render_pts = std::vector<cv::Point2f>();
                cv::projectPoints(vis_points, rvec, tvec, camera_matrix, dist_matrix, render_pts);
                cv::line(frame, render_pts.at(0), render_pts.at(2), cv::Scalar(255,0,255));
                //printf("Tag X: %f Y %f Z %f\n", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                //printf("tag detected");
        }
        cond.notify_all();
        apriltag_detections_destroy(detections);

        

        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);            
        /*printf("Capture: %fms,%fms\n", 
            millisecondDiff(start_time, cap_time),
            millisecondDiff(cap_time, end_time)
        );*/
        if(millisecondDiff(start_time, end_time) < 16.66) {
            usleep((int)(1000.0*(16.66 - millisecondDiff(start_time, end_time))));
        }
    }
    apriltag_detector_destroy(td);
}

namespace fs=std::filesystem;

// hi
int main(int, char**) {
    /*std::string camPath = "/sys/bus/usb/devices/2-1:1.0/video4linux";
    int minNum = -1;
    for (const auto & entry : fs::directory_iterator(camPath)) {
        int num = -1;
        sscanf(entry.path().filename().c_str(), "video%i", &num);
        if(num >= 0) {
            printf("found video device %i\n", num);
            if(minNum < 0) minNum = num;
            if(num < minNum) minNum = num;
        }
    }
    printf("using /dev/video%i\n", minNum);*/
    cv::VideoCapture cap;

    int apiID = cv::CAP_V4L2;
    cap.open("/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_1F9F07DF-video-index0", apiID);

    apriltag(cap);
}
