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
#include "posestreamerserver.hpp"

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

    u_int32_t codec = 0x47504A4D;
    //u_int32_t codec = 'YUYV';
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_FOURCC, codec);
    cap.set(cv::CAP_PROP_EXPOSURE, 30);
    //cap.set(cv::CAP_PROP_FOCUS, 255);
    //cap.set(cv::CAP_PROP_AUTOFOCUS, 1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 120);

    if(!cap.isOpened()) {
        cerr << "Unable to open camera\n";
        return -1;
    }

    std::condition_variable cond;
    vision::Streamer smr = vision::Streamer("maincam", 8008, &frame, &cond);

    printf("starting poss");
    posestreamer::PoseStreamerServer pss = posestreamer::PoseStreamerServer(8833);

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

    //for arducam
    float cam[10] = {546.68576068,   0,         293.5576285,
                    0,         547.1991467,  229.61722473,
                    0,           0,           1,        };
    cv::Mat camera_matrix = cv::Mat(3,3,5, cam);

    //for c920
    //float dist[5] = {0.17413211, -0.47767714, -0.00792523, -0.00707075, 0.60882555};

    //for arducam
    float dist[5] = { 0.08148129, -0.21510129, -0.00484131,  0.00231954,  0.41710648};

    cv::Mat dist_matrix = cv::Mat(cv::Size(1,5), 5, dist);

    double tagw = 165;

    std::vector<cv::Point3d> tag_points = std::vector<cv::Point3d>();

    tag_points.push_back(cv::Point3d(0,0,0));
    tag_points.push_back(cv::Point3d(165,0,0));
    tag_points.push_back(cv::Point3d(165,165,0));
    tag_points.push_back(cv::Point3d(0,165,0));

    std::map<int, std::vector<cv::Point3d>> tag_map;
    tag_map[2].push_back(cv::Point3d(0,0,1355));
    tag_map[2].push_back(cv::Point3d(0+tagw,0,1355));
    tag_map[2].push_back(cv::Point3d(0+tagw,0,1355+tagw));
    tag_map[2].push_back(cv::Point3d(0,0,1355+tagw));
    
    tag_map[3].push_back(cv::Point3d(655,0,1355));
    tag_map[3].push_back(cv::Point3d(655+tagw,0,1355));
    tag_map[3].push_back(cv::Point3d(655+tagw,0,1355+tagw));
    tag_map[3].push_back(cv::Point3d(655,0,1355+tagw));

    std::vector<cv::Point3f> vis_points = std::vector<cv::Point3f>();

    vis_points.push_back(cv::Point3f(0,0,0));
    vis_points.push_back(cv::Point3f(165,0,0));
    vis_points.push_back(cv::Point3f(165,165,0));
    vis_points.push_back(cv::Point3f(0,165,0));

    for(;;) {
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
        cap.grab();
        cap.retrieve(frame);
        
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cap_time);

        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
        
        zarray_t *detections = apriltag_detector_detect(td, &im);
        std::vector<cv::Point2f> det_points = std::vector<cv::Point2f>();
        std::vector<cv::Point3d> model_points = std::vector<cv::Point3d>();

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            

            if(tag_map.count(det->id) > 0) {
                //add the points from the field model and the corresponding identified aprilTag
                det_points.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
                det_points.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
                det_points.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
                det_points.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));
                cv::cornerSubPix(gray, det_points, cv::Size(6,6), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 ));

                model_points.push_back(tag_map.at(det->id).at(0));
                model_points.push_back(tag_map.at(det->id).at(1));
                model_points.push_back(tag_map.at(det->id).at(2));
                model_points.push_back(tag_map.at(det->id).at(3));
            }


            //draw the edges of the marker on the video stream
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
        }

        if(det_points.size() >1) {
            cv::Mat rvec, tvec, rmatrix;
            cv::solvePnP(model_points, det_points, camera_matrix, dist_matrix, rvec, tvec);

            cv::Rodrigues(rvec, rmatrix);
            cv::Mat cam_pose = tvec.reshape(0, 1) * rmatrix;

            std::vector<cv::Point2f> render_pts = std::vector<cv::Point2f>();
            cv::projectPoints(vis_points, rvec, tvec, camera_matrix, dist_matrix, render_pts);
            cv::line(frame, render_pts.at(0), render_pts.at(2), cv::Scalar(255,0,255));
            printf("Cam X: %f Y %f Z %f\n", -cam_pose.at<double>(0), cam_pose.at<double>(1), cam_pose.at<double>(2));
            cv::rectangle(frame, cv::Rect(0, 0,100, 100), cv::Scalar(0,0,0),-1);
            cv::line(frame, cv::Point(50,0), cv::Point((cam_pose.at<double>(0)/-20)+50, (cam_pose.at<double>(1)/20)),cv::Scalar(255, 255, 255), 2);

            //printf("rmat type: %i\n", rmatrix.type());
            double pt[3] = {200, 0, 500};
            cv::Mat cam_2nd_point = (cv::Mat(cv::Size(3, 1), 6, pt) * rmatrix) + cam_pose;
            printf("X: %f Y: %f Z: %f\n", cam_2nd_point.at<double>(0), cam_2nd_point.at<double>(1), cam_2nd_point.at<double>(2));
            cv::line(frame, cv::Point(50, 50),cv::Point(50+(cam_2nd_point.at<double>(0)/-20), (cam_2nd_point.at<double>(1)/20)), cv::Scalar(255, 0, 255));

            //cv::drawFrameAxes(frame, camera_matrix,dist_matrix, rvec, cam_2nd_point, 200);

            vector<double> cam_pose_vec;
            cam_pose_vec.push_back(cam_pose.at<double>(0));
            cam_pose_vec.push_back(cam_pose.at<double>(1));
            cam_pose_vec.push_back(cam_pose.at<double>(2));

            pss.publish_stream(1, 1, cam_pose_vec);
        }

        cond.notify_all();
        apriltag_detections_destroy(detections);

        

        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);            
        printf("Capture: %fms,%fms\n", 
            millisecondDiff(start_time, cap_time),
            millisecondDiff(cap_time, end_time)
        );
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
    cap.open("/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0", apiID);

    apriltag(cap);
}
