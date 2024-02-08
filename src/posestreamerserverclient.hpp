#include <stdlib.h>
#include <vector>
#include <map>
#include <mutex>
#include <thread>

namespace posestreamer
{

class PoseStreamerServerClient
{
private:
    void process_packet();
    void handle_request(size_t header_len);
    void send_request_response(char type, char id, char status);
    std::map<int, int> requested_streams;
    std::mutex socket_mutex;
    int socket;
    std::thread server_thread;
    bool m_valid;
public:
    bool isValid();
    PoseStreamerServerClient(int sock);
    std::vector<int> getRequestedStreams();
    void publishStream(int pose_class, int obj_id, std::vector<double> pose);
    ~PoseStreamerServerClient();
};
}

