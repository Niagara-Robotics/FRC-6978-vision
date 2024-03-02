
#include "utils.hpp"
#include "streamer.hpp"
#include "posestreamerserver.hpp"
#include "tagmap.h"
#include <thread>

#include <apriltagpipeline.hpp>
#include <notepipeline.hpp>

using namespace std;




namespace fs=std::filesystem;

// hi
int main(int, char**) {
    posestreamer::PoseStreamerServer pss = posestreamer::PoseStreamerServer(8833);

    vision::AprilTagPipeline reverse_camera_pipeline = vision::AprilTagPipeline();
    reverse_camera_pipeline.device = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0";
    reverse_camera_pipeline.pose_streamer = &pss;
    reverse_camera_pipeline.video_streamer_port = 8008;

    std::thread reverse_camera_pipeline_thread = std::thread(&vision::AprilTagPipeline::execute, &reverse_camera_pipeline);

    vision::NotePipeline note_pipeline = vision::NotePipeline();
    note_pipeline.video_streamer_port = 8009;
    note_pipeline.device = "/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0";

    std::thread note_camera_pipeline_thread = std::thread(&vision::NotePipeline::execute, &note_pipeline);

    while(true) {
    }
}
