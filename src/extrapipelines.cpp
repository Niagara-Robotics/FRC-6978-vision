#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <time.h>

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
