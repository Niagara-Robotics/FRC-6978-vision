#include <cpp-httplib/httplib.h>
#include <opencv2/core.hpp>

namespace vision {
    class Streamer
    {
    private:
        httplib::Server svr;
        std::string name;
        std::thread thread;
        int port;
        cv::Mat *frame;
        std::mutex *frame_mutex;
        void serve();
    public:
        Streamer(std::string name, int port, cv::Mat *frame, std::condition_variable *frame_mutex);
        ~Streamer();
    };
}