
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

int parse(int, char**) {
    FILE *fp = fopen("pipelines.conf", "r");

    char s1[255];
    char s2[255];
    char s3[255];
    char s4[255];

    char line[255];

    int r = 0;

    while(true) {
        r = fscanf(fp, "%s\n", line);

        if(r == EOF) {
            printf("EOF\n");
            break;
        }

        //printf("Line: %s\n", line);

        r = sscanf(line, "%[^=\n]=%s\n", s3, s4);

        if(r == 2) {
            printf("config value: %s, %s\n", s3, s4);
        } else if(r == 1) {
            //printf("attempting to parse section header\n");
            sscanf(line, "[%[^:]:%[^]]\]\n", s1, s2);
            printf("got section header: %s %s\n", s1, s2);
        } else if (r == EOF) {
            break;
        }
    }
    printf("1: %s 2: %s 3: %s 4: %s\n", s1, s2, s3, s4);
}

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
    note_pipeline.pose_streamer = &pss;

    std::thread note_camera_pipeline_thread = std::thread(&vision::NotePipeline::execute, &note_pipeline);

    while(true) {
    }
}
