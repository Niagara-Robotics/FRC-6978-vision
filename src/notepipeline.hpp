#include <visionpipeline.hpp>
#include <string>
#include <posestreamerserver.hpp>

#ifndef NOTE_PIPELINE_H

namespace vision {
    class NotePipeline {
        
        public:
        std::string name;
        std::string device;

        posestreamer::PoseStreamerServer *pose_streamer;
        int video_streamer_port;

        void execute();
    };
}

#define NOTE_PIPELINE_H
#endif