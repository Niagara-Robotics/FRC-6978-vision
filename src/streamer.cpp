#include <streamer.hpp>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "utils.hpp"
//hello
namespace vision
{
    Streamer::Streamer(std::string name, int port, cv::Mat *frame, std::condition_variable *frame_mutex)
    {
        this->handler_index = svr.get_handlers_.size;
        svr.Get("/hi", [frame, frame_mutex, this](const httplib::Request &req, httplib::Response &res) {
            
            /*res.set_header("Connection", "close");
            res.set_header("Max-Age", "0");
            res.set_header("Expires", "0");
            res.set_header("Cache-Control", "no-cache, private");
            res.set_header("Pragma", "no-cache");
            res.set_header("Content-Type", "multipart/x-mixed-replace;boundary=--boundary");
            */
            printf("test");
            res.stm->write("HTTP/1.1 200 OK\r\n");
            res.stm->write("Connection: close\r\n");
            res.stm->write("Max-Age: 0\r\n");
            res.stm->write("Expires: 0\r\n");
            res.stm->write("Cache-Control: no-cache, private\r\n");
            res.stm->write("Pragma: no-cache\r\n");
            res.stm->write("Content-Type: multipart/x-mixed-replace;boundary=--boundary\r\n\r\n");

//sls
            struct timespec start_time, mutex_time, encode_time, network_time;

            std::vector<uchar> buf = std::vector<uchar>();
            std::mutex m;
            std::unique_lock<std::mutex> lk{m};

            cv::Mat my_frame;

            while(true) {
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
                frame_mutex->wait(lk);
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &mutex_time);
                my_frame = frame->clone();
                cv::imencode(".jpg", my_frame, buf);
                std::string image (buf.begin(), buf.end());
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &encode_time);
                std::string imgheader = "--boundary\r\n"
                        "Content-Type: image/jpeg\r\n"
                        "Content-Length: " +
                        std::to_string(image.size()) +
                        "\r\n\r\n" + image;
                if(res.stm->write(imgheader) != (long int)imgheader.size()) {
                    printf("closed stream");
                    return;
                }
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &network_time);

                if(!this->m_valid) return;

                /*printf("Stream: %fms,%fms,%fms\n", 
                    millisecondDiff(start_time, mutex_time),
                    millisecondDiff(mutex_time, encode_time),
                    millisecondDiff(encode_time, network_time)
                    //millisecondDiff(contour_time, filter_time)
                );
                /*if(millisecondDiff(start_time, network_time) < 20) {
                    usleep((int)(1000.0*(20 - millisecondDiff(start_time, network_time))));
                    printf("buffering: %i", (int)(1000.0*(20 - millisecondDiff(start_time, network_time))));
                }*///
            }
        });
        this->name = name;
        this->port = port;
        this->frame = frame;

        this->m_valid = true;
        thread = std::thread(&Streamer::serve, this);
        
    }

    void Streamer::serve() {
        svr.listen("0.0.0.0", port);
    }
    
    Streamer::~Streamer()
    {
        m_valid = false;
        svr.get_handlers_.erase(this->handler_index);
        svr.stop();
        if(thread.joinable()) {
            thread.join();
        }
    }
    
} // namespace vision
