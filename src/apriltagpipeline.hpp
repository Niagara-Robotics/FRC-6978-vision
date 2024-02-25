#include <visionpipeline.h>
#include <posestreamerserver.hpp>

#ifndef APRILTAG_PIPELINE_H

int apriltag_pipeline_execute(std::string dev, posestreamer::PoseStreamerServer *pose_streamer, int video_streamer_port);

#define APRILTAG_PIPELINE_H
#endif