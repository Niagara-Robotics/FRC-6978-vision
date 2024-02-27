#include <visionpipeline.h>
#include <posestreamerserver.hpp>

#include <opencv2/core.h>
#include <string.h>

#ifndef APRILTAG_PIPELINE_H

int apriltag_pipeline_execute(std::string dev, posestreamer::PoseStreamerServer *pose_streamer, int video_streamer_port);

namespace vision {
    class AprilTagPipeline: VisionPipeline {
        private:
            vision::Streamer webStreamer;

        public:
            string name;

            int safe_zone = 5;
            float decimate = 2.0;
            int corner_refine_iteration_limit = 250;
            int pose_refine_iteration_limit = 250;

            cv::Mat camera_matrix, distortion_matrix;

            void execute();
    }
}

#define APRILTAG_PIPELINE_H
#endif
