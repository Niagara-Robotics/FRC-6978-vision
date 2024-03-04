#include <visionpipeline.hpp>
#include <posestreamerserver.hpp>

#include <opencv2/core.hpp>
#include <string.h>
#include <thread>

#ifndef APRILTAG_PIPELINE_H

//int apriltag_pipeline_execute(std::string dev, posestreamer::PoseStreamerServer *pose_streamer, int video_streamer_port);

namespace vision {
    class AprilTagPipeline {
        private:
            //vision::Streamer webStreamer;
            //void AprilTagPipeline::set_properties(cv::VideoCapture cap);
        public:
            std::string name;
            std::string device;

            posestreamer::PoseStreamerServer *pose_streamer;
            int video_streamer_port;

            int safe_zone = 10;
            float decimate = 2.0;
            float sigma = 1.55;
            int corner_refine_iteration_limit = 250;
            int pose_refine_iteration_limit = 250;
            int exposure = 11;

            float target_z = 800.0;

            cv::Mat camera_matrix, distortion_matrix;

            cv::Mat frame;

            bool processing_ready = false;
            bool frame_waiting = false;
            long expose_ts, last_frame_ts;

            std::thread capture_thread;
            std::thread processing_thread;

            void start();
            void execute();
            void capture();

            AprilTagPipeline();
            ~AprilTagPipeline();
    };
}

#define APRILTAG_PIPELINE_H
#endif
