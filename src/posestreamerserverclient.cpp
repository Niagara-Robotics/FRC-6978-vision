#include "posestreamerpackets.hpp"
#include "posestreamerserverclient.hpp"

#include <stdio.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <istream>
#include <thread>

namespace posestreamer
{

void PoseStreamerServerClient::process_packet() 
{
    while (true)
    {
        
        size_t in;
        int packet_type = 0;
        in = read(socket, &packet_type, 1);
        if(in<1) {
            m_valid=false;
            return;
        }
        //socket_mutex.lock();
        printf("packet_type: %i\n", packet_type);

        size_t header_len = 0;
        in = read(socket, &header_len, 1);
        if(in<1) {
            m_valid = false;
            return;
        }

        switch (packet_type)
        {
        case PACKET_TYPE_REQUEST:
            handle_request(header_len);
            break;
        
        default:
            break;
        }
        //socket_mutex.unlock();
        printf("done receiving packet\n");
    }
}

void PoseStreamerServerClient::send_request_response(char type, char id, char status) {
    packetheader header;
    header.packet_type = PACKET_TYPE_REQUEST_RESPONSE;
    header.header_size = sizeof(requestresponse);

    requestresponse response;
    response.request_type = type;
    response.request_id = id;
    response.status_code = status;

    socket_mutex.lock();
    size_t out = 0;

    out = write(socket, &header, sizeof(header));
    if(out < sizeof(header)) m_valid=false;

    out = write(socket, &response, sizeof(response));
    if(out < sizeof(response)) m_valid=false;

    socket_mutex.unlock();
}

void PoseStreamerServerClient::handle_request(size_t header_len) {
    posestreamer::requestpacket packet = posestreamer::requestpacket();
    size_t in;
    //todo: length checking
    packet.request_header = posestreamer::requestheader();
    in = read(socket, &packet.request_header, header_len);
    if(in < header_len)  {
        printf("Failed receiving header: %i\n", in);
    }

    packet.request_body = malloc(packet.request_header.request_size);
    in = read(socket, packet.request_body, packet.request_header.request_size);
    if(in < packet.request_header.request_size) return;

    switch (packet.request_header.request_type)
    {
    case REQUEST_TYPE_POSE_STREAM_INIT:
        requested_streams[((uint8_t*)(packet.request_body))[0]] = ((uint8_t*)(packet.request_body))[1];
        printf("Client requested stream for pose_class %i, object ID %i\n", ((uint8_t*)(packet.request_body))[0], ((uint8_t*)(packet.request_body))[1]);
        send_request_response(packet.request_header.request_type, packet.request_header.request_id, 0);
        break;
    
    default:
        break;
    }
}

void PoseStreamerServerClient::publishStream(int pose_class, int obj_id, std::vector<double> pose) {
    int requested_obj_id;
    try
    {
        requested_obj_id = requested_streams.at(pose_class);
    }
    catch(const std::exception& e)
    {
        return;
    }

    if(requested_obj_id != 0 && obj_id != requested_obj_id) {
        return;
    }

    posestreamer::streamheader stream_header;
    stream_header.object_id = obj_id;
    stream_header.pose_class = pose_class;
    stream_header.blob_size = 8*pose.size();

    posestreamer::packetheader packet_header;
    packet_header.packet_type = PACKET_TYPE_STREAM;
    packet_header.header_size = sizeof(stream_header);


    socket_mutex.lock();

    size_t out;

    out = write(socket, &packet_header, sizeof(packet_header));
    if(out < sizeof(packet_header)) m_valid=false;

    out = write(socket, &stream_header, sizeof(stream_header));
    if(out < sizeof(stream_header)) m_valid=false;

    for (size_t i = 0; i < pose.size(); i++)
    {
        char *bytearray = reinterpret_cast<char*>(&pose.at(i));
        out = write(socket, bytearray, sizeof(bytearray));
        if(out < sizeof(bytearray)) printf("failed to write double to stream");
    }
    socket_mutex.unlock();
}

bool PoseStreamerServerClient::isValid() {
    return m_valid;
}

PoseStreamerServerClient::PoseStreamerServerClient(int sock)
{
    m_valid = true;
    socket = sock;
    printf("Created client object\n");
    server_thread = std::thread(&PoseStreamerServerClient::process_packet, this);
}

PoseStreamerServerClient::~PoseStreamerServerClient() {
    if(server_thread.joinable()) {
        server_thread.join();
    }
    close(socket);
    printf("Closed client connection\n");
}

}