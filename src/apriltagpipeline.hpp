#include <visionpipeline.hpp>
#include <posestreamerserver.hpp>

#include <opencv2/core.hpp>
#include <string.h>

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

            int safe_zone = 5;
            float decimate = 2.0;
            float sigma = 1.55;
            int corner_refine_iteration_limit = 250;
            int pose_refine_iteration_limit = 250;
            int exposure = 11;

            cv::Mat camera_matrix, distortion_matrix;

            void execute();

            AprilTagPipeline();
            ~AprilTagPipeline();
    };
}

#define APRILTAG_PIPELINE_H
#endif
