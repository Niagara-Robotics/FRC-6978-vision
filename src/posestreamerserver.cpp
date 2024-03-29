#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <unistd.h>

#include <posestreamerserver.hpp>
#include <stdio.h>
//#include <posestreamerserverclient.hpp>

namespace posestreamer
{
    void PoseStreamerServer::main_loop() {
        sockaddr client_addr;
        int client_addr_len = sizeof(client_addr);

        while (true){
            printf("waiting for client\n");
            if(!m_valid) return;
            int client_socket = accept(server_socket, &client_addr, (socklen_t*)&client_addr_len);
            if(client_socket < 0) {
                printf("Failed to accept connection\n");
                continue;
            }
            printf("accepted connection\n");

            clients.push_back(std::unique_ptr<PoseStreamerServerClient>(new PoseStreamerServerClient(client_socket)));
        }
    }

    void PoseStreamerServer::publish_stream(int pose_class, int obj_id, long timestamp, std::vector<double> pose) {
        //printf("publishing stream %i, %i", pose_class, obj_id);
        for (std::vector<std::unique_ptr<PoseStreamerServerClient>>::iterator i = clients.begin(); i < clients.end(); i++)
        {
            if(!i->get()->isValid()) {
                printf("Removed client\n");
                clients.erase(i);
            }
            i->get()->publishStream(pose_class, obj_id, timestamp, pose);
        }
    }
    
    PoseStreamerServer::PoseStreamerServer(int port) {
        sockaddr_in server_addr;
        printf("starting posestreamer\n");
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if(server_socket == -1) {
            printf("Failed to create socket\n");
            delete this;
            return;
        }
        
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(port);

        if((bind(server_socket, (sockaddr*)&server_addr, sizeof(server_addr))) != 0) {
            printf("Failed to bind socket\n");
            delete this;
            return;
        }

        if((listen(server_socket, 5)) != 0) {
            printf("Failed to start listening\n");
            delete this;
            return;
        }
        printf("starting thread\n");
        m_valid = true;
        thread = std::thread(&PoseStreamerServer::main_loop, this);
    }

    PoseStreamerServer::~PoseStreamerServer() {
        printf("Closing socket\n");
        close(server_socket);
        m_valid = false;
        if(thread.joinable()) {
            thread.join();
        }
    }
} // namespace posestreamer
