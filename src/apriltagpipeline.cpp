#include <opencv2/core.hpp>
#include <opencv2/core/hal/hal.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <filesystem>
#include <vector>
#include <stdlib.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include "utils.hpp"
#include "streamer.hpp"
#include "posestreamerserver.hpp"
#include "tagmap.h"
#include "apriltagpipeline.hpp"
#include <visionpipeline.hpp>


using namespace std;
void set_properties(cv::VideoCapture cap) {
    u_int32_t codec = 0x47504A4D; //MJPEG

    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_FOURCC, codec);
    cap.set(cv::CAP_PROP_EXPOSURE, 10); //25 for arducam
    //cap.set(cv::CAP_PROP_AUTOFOCUS, 1);
    //cap.set(cv::CAP_PROP_FOCUS, 250);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    cap.set(cv::CAP_PROP_FPS, 120);
}

double straight_line_distance(cv::Mat poseA, cv::Mat poseB) {
    double dX = pow(abs(poseB.at<double>(0) - poseA.at<double>(0)), 2);
    double dY = pow(abs(poseB.at<double>(1) - poseA.at<double>(1)), 2);
    double dZ = pow(abs(poseB.at<double>(2) - poseA.at<double>(2)), 2);

    return sqrt(dX + dY + dZ);
}

namespace vision {

    

    void AprilTagPipeline::execute()
    {
        cv::Mat gray;

        

        std::condition_variable cond;
        vision::Streamer smr = vision::Streamer("maincam", video_streamer_port, &frame, &cond);

        printf("starting poss");

        struct timespec start_time, cap_time, convert_time, tag_time, model_time, end_time;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);

        apriltag_detector_t *td = apriltag_detector_create();
        apriltag_family_t *tf = tag36h11_create();
        apriltag_detector_add_family(td, tf);

        td->quad_decimate = decimate;
        td->quad_sigma = sigma;
        printf("Default params: decimate: %f, sigma: %f, threads: %f\n", td->quad_decimate, td->quad_sigma, td->nthreads);
        td->nthreads = 4;

        //int safe_zone = 5; //border within which tags are discarded

        //for arducam
        /*float cam[10] = {546.68576068,   0,         293.5576285,
                        0,         547.1991467,  229.61722473,
                        0,           0,           1,        };*/
        float cam[10] = {901.56023498,  0.0,           600.07768005,
                        0.0,          900.4374398,   397.90081813,
                        0.0,          0.0,           1.0        };
        cv::Mat camera_matrix = cv::Mat(3,3,5, cam);

        //for c920
        //float dist[5] = {0.17413211, -0.47767714, -0.00792523, -0.00707075, 0.60882555};

        //for arducam
        float dist[5] = { 0.05925655, -0.10029338, -0.00188174, -0.00290582,  0.05352116};
        cv::Mat dist_matrix = cv::Mat(cv::Size(1,5), 5, dist);

        std::map<int, std::vector<cv::Point3d>> tag_map = build_map();

        double inv[3] = {-1.0, -1.0, -1.0};
        cv::Mat inv_matrix = cv::Mat(cv::Size(3,1), CV_64F, inv);

        printf("Size: %i, %i\n", inv_matrix.cols, inv_matrix.rows);

        cv::Mat robot_pose, previous_robot_pose;

        for(;;) {
            processing_ready = true;
            while(!frame_waiting) {usleep(1);}
            frame_waiting =false;
            processing_ready = false;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);

            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cap_time);

            cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &convert_time);

            zarray_t *detections = apriltag_detector_detect(td, &im);
            std::vector<cv::Point2f> det_points = std::vector<cv::Point2f>();
            std::vector<cv::Point3d> model_points = std::vector<cv::Point3d>();

            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tag_time);

            vector<cv::Mat> rvecs, tvecs;

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                //check if tag corners infringe upon frame edge safe zone
                bool reject = false;
                for(int corner=0; corner < 4; corner++) {
                    if(det->p[corner][0] < safe_zone) reject = true;
                    if(det->p[corner][0] > gray.cols - safe_zone - 1) reject = true;
                    if(det->p[corner][1] < safe_zone) reject = true;
                    if(det->p[corner][1] > gray.rows - safe_zone - 1) reject = true;
                }

                if(reject) continue;

                if(tag_map.count(det->id) > 0) {
                    //add the points from the field model and the corresponding identified aprilTag
                    det_points.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
                    det_points.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
                    det_points.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
                    det_points.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));
                    //det_points.push_back(cv::Point2f(det->c[0], det->c[1])); //center

                    model_points.push_back(tag_map.at(det->id).at(0));
                    model_points.push_back(tag_map.at(det->id).at(1));
                    model_points.push_back(tag_map.at(det->id).at(2));
                    model_points.push_back(tag_map.at(det->id).at(3));
                    //model_points.push_back(tag_map.at(det->id).at(4)); //center

                    vector<double> tag_pose_vec;
                    tag_pose_vec.push_back(det->c[0]);
                    tag_pose_vec.push_back(det->c[1]);
                    pose_streamer->publish_stream(2, det->id, expose_ts, tag_pose_vec);
                }


                //draw the edges of the marker on the video stream
                /*cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
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
                        cv::Scalar(0xff, 0, 0), 2);*/
            }

            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &model_time);

            if(det_points.size() >1) {
                //cv::cornerSubPix(gray, det_points, cv::Size(4,4), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 ));

                for(int i = 0; i < det_points.size(); i++)
                cv::drawMarker(frame, det_points.at(i), cv::Scalar(0xff, 0, 0), cv::MARKER_CROSS, 20, 1);

                cv::Mat potential_rmatrix_1, potential_rmatrix_2, rmatrix, proj_err, potential_pose_1, potential_pose_2;
                //cv::solvePnP(model_points, det_points, camera_matrix, dist_matrix, rvec, tvec);
                cv::solvePnPGeneric(model_points, det_points, camera_matrix, dist_matrix, rvecs, tvecs, false, cv::SOLVEPNP_SQPNP,cv::noArray(), cv::noArray(), proj_err);

                //cv::solvePnPRefineLM(model_points, det_points, camera_matrix, dist_matrix, rvecs.at(0), tvecs.at(0));
                cv::Rodrigues(rvecs.at(0), potential_rmatrix_1);
                potential_pose_1 = tvecs.at(0).reshape(0, 1) * potential_rmatrix_1;
                //printf("Size: %i, %i\n", potential_pose_1.cols, potential_pose_1.rows);
                potential_pose_1 = potential_pose_1.mul(inv_matrix);


                double pose_1_score = 0.0;
                double pose_2_score = 0.0;

                if(tvecs.size() > 1) { //decide which pose to use in cases of ambiguous solutions
                    //cv::solvePnPRefineLM(model_points, det_points, camera_matrix, dist_matrix, rvecs.at(1), tvecs.at(1));
                    cv::Rodrigues(rvecs.at(1), potential_rmatrix_2);
                    potential_pose_2 = tvecs.at(1).reshape(0, 1) * potential_rmatrix_2;
                    potential_pose_2 = potential_pose_2.mul(inv_matrix);

                    pose_1_score -= abs((potential_pose_1.at<double>(2)) - target_z) / 100.0; //subtract 1 score for every 100mm deviation from expected Z height
                    pose_2_score -= abs((potential_pose_2.at<double>(2)) - target_z) / 100.0;

                    if(previous_robot_pose.cols != 0) { //if there is a previous pose
                        //add score to the closes pose to the previous pose
                        pose_1_score -= straight_line_distance(potential_pose_1, previous_robot_pose) / 300; // subtract 1 score for every 300mm deviation from the previous pose
                        pose_2_score -= straight_line_distance(potential_pose_2, previous_robot_pose) / 300;
                    }

                    if(pose_1_score > pose_2_score) {
                        potential_pose_1.copyTo(robot_pose);
                        potential_rmatrix_1.copyTo(rmatrix);
                    } else {
                        potential_pose_2.copyTo(robot_pose);
                        potential_rmatrix_2.copyTo(rmatrix);
                    }
                } else {
                    potential_pose_1.copyTo(robot_pose);
                    potential_rmatrix_1.copyTo(rmatrix);
                }

                

                //use a test point(straight down the camera's optical axis) to calculate the robot's heading
                double pt[3] = {0, 0, 250};
                cv::Mat cam_2nd_point = (cv::Mat(cv::Size(3, 1), 6, pt) * rmatrix);

                //calculate heading
                double heading = atan2(cam_2nd_point.at<double>(1), cam_2nd_point.at<double>(0));

                vector<double> cam_pose_vec;
                cam_pose_vec.push_back(robot_pose.at<double>(0));
                cam_pose_vec.push_back(robot_pose.at<double>(1));
                cam_pose_vec.push_back(robot_pose.at<double>(2));
                cam_pose_vec.push_back(heading);
                cam_pose_vec.push_back(proj_err.at<double>(0)); //reprojection error
                cam_pose_vec.push_back(det_points.size() / 4.0); //number of detected tags
                cam_pose_vec.push_back((double)tvecs.size()); //number of solutions

                robot_pose.copyTo(previous_robot_pose);

                pose_streamer->publish_stream(1, 1, expose_ts, cam_pose_vec);
            }

            cond.notify_all();
            apriltag_detections_destroy(detections);

            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);
            /*printf("Timing: %fms,%fms,%fms,%fms,%fms total: %fms\n",
                millisecondDiff(start_time, cap_time), //cap
                millisecondDiff(cap_time, convert_time), //convert
                millisecondDiff(convert_time, tag_time), //tags
                millisecondDiff(tag_time, model_time), //model
                millisecondDiff(model_time, end_time), //solve
                millisecondDiff(cap_time, end_time) //total
            );*/
            if(millisecondDiff(start_time, end_time) < 16.66) {
                usleep((int)(1000.0*(16.66 - millisecondDiff(start_time, end_time))));
            }
        }
        apriltag_detector_destroy(td);
    }

    void AprilTagPipeline::capture() {
        cv::VideoCapture cap;

        cap.open(device, cv::VideoCaptureAPIs::CAP_V4L2);

        set_properties(cap);

        if(!cap.isOpened()) {
            cerr << "Unable to open camera\n";
            return;
        }
        for(;;) {
            
            if(!cap.grab()) {
                cap.open(device, cv::CAP_V4L2);
                if(!cap.isOpened()) continue;
                set_properties(cap);
                continue;
            }
            long frame_ts = CurrentTime_nanoseconds();
            vector<double> tele = vector<double>();
            tele.push_back((frame_ts - last_frame_ts) / 1000000.0);
            long last_frame_ts=frame_ts;
            pose_streamer->publish_stream(128, 1, frame_ts, tele);
            if(processing_ready) {
                expose_ts = frame_ts;
                cap.retrieve(frame);
                frame_waiting = true;
                processing_ready = false;
            }
        }
    }

    void AprilTagPipeline::start() {
        capture_thread = std::thread(&vision::AprilTagPipeline::capture, this);
        processing_thread = std::thread(&vision::AprilTagPipeline::execute, this);
    }

    AprilTagPipeline::AprilTagPipeline() {

    }

    AprilTagPipeline::~AprilTagPipeline() {}
}