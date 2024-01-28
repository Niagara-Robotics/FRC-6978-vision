#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <time.h>

using namespace std;

int main(int, char**)
{
    cv::Mat frame, hsv, threshold, eroded;
    cv::VideoCapture cap;

    int deviceID = 3;
    int apiID = cv::CAP_ANY;
    cap.open(deviceID, apiID);
    u_int32_t codec = 0x47504A4D;
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

    struct timespec start_time, end_time;

    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = 4;
    cv::Mat erosion_kernel = cv::getStructuringElement( erosion_type,
                    cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                    cv::Point( erosion_size, erosion_size ) );
    vector<cv::Vec4i> hierarchy;
    vector<vector<cv::Point>> contours;
    for(;;) {
        cap.read(frame);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 130, 60), cv::Scalar(21,255,255), threshold);
        cv::erode(threshold, eroded, erosion_kernel);

        cv::findContours(eroded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
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

        

        if(largestIndex >= 0) {
            cv::Rect box = cv::boundingRect(contours[largestIndex]);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);
            long diffInNanos = (end_time.tv_sec - start_time.tv_sec) * (long)1e9 + (end_time.tv_nsec - start_time.tv_nsec);
            double diffMs = (double)diffInNanos / 1000000.0;
            printf("X: %i Y: %i Time: %f\n", box.x, box.y, diffMs);
        }
        
    }
}