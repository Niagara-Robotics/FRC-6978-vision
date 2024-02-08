#include <vector>
#include <list>
#include <posestreamerserverclient.hpp>
#include <thread>

namespace posestreamer
{
    class PoseStreamerServer
    {
    private:
        std::vector<std::unique_ptr<PoseStreamerServerClient>> clients;
        int server_socket;
        void main_loop();
        std::thread thread;
    public:
        PoseStreamerServer(int port);
        void publish_stream(int pose_class, int obj_id, std::vector<double> pose);
        ~PoseStreamerServer();
    };
    
} // namespace posestreamer
