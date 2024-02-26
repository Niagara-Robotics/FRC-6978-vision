
#include "utils.hpp"
#include "streamer.hpp"
#include "posestreamerserver.hpp"
#include "tagmap.h"

#include <apriltagpipeline.hpp>

using namespace std;




namespace fs=std::filesystem;

// hi
int main(int, char**) {
    posestreamer::PoseStreamerServer pss = posestreamer::PoseStreamerServer(8833);

    for(;;) {
        printf("opening");

        apriltag_pipeline_execute("/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0", &pss, 8008);
        //apriltag_pipeline_execute("/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_1F9F07DF-video-index0", &pss, 8008);
        printf("retrying");
    }
}
