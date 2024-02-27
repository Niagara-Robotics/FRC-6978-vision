#include "posestreamerpackets.hpp"
#include "posestreamerserverclient.hpp"
#include "utils.hpp"

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

        int preamble_1 = 0;
        in = read(socket, &preamble_1, 1);
        if(in<1) {
            m_valid=false;
            return;
        }
        if(preamble_1 != PACKET_PREAMBLE_1) {
            printf("Skipping non-preamble byte\n");
            continue;
        }

        int preamble_2 = 0;
        in = read(socket, &preamble_2, 1);
        if(in<1) {
            m_valid=false;
            return;
        }
        if(preamble_2 != PACKET_PREAMBLE_2) {
            printf("Skipping non-preamble 2 byte\n");
            continue;
        }

        int packet_type = 0;
        in = read(socket, &packet_type, 1);
        if(in<1) {
            m_valid=false;
            return;
        }
        //socket_mutex.lock();
        if(packet_type != 3) {
            printf("packet_type: %i\n", packet_type);
        }
        if(packet_type == 0) {
            continue;
        }

        size_t header_len = 0;
        in = read(socket, &header_len, 1);
        if(in<1) {
            m_valid = false;
            return;
        }

        //printf("header_len: %i\n", header_len);


        switch (packet_type)
        {
        case PACKET_TYPE_REQUEST:
            printf("Got request\n");
            handle_request(header_len);
            break;
        case PACKET_TYPE_CLOCK_REQUEST:
            handle_clock_request();
            break;
        default:
            break;
        }
        //socket_mutex.unlock();
        //printf("done receiving packet\n");
    }
}

void PoseStreamerServerClient::send_request_response(char type, char id, char status) {
    packetpreamble preamble;
    
    packetheader header;
    header.packet_type = PACKET_TYPE_REQUEST_RESPONSE;
    header.header_size = sizeof(requestresponse);

    requestresponse response;
    response.request_type = type;
    response.request_id = id;
    response.status_code = status;

    socket_mutex.lock();
    size_t out = 0;

    out = write(socket, &preamble, sizeof(preamble));
    if(out < sizeof(preamble)) m_valid=false;

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
    size_t recvdHeaderLen = 0;
    while (recvdHeaderLen < header_len) {
        in = read(socket, &packet.request_header, header_len);
        if(in < 1) {
            printf("Failed receiving header: expected %i got %i\n", header_len, in);
            return;
        }
        recvdHeaderLen+= in;
    }
    if(recvdHeaderLen < header_len)  {
        printf("Header too short: expected %i got %i\n", header_len, recvdHeaderLen);
        return;
    }

    packet.request_body = malloc(packet.request_header.request_size);
    size_t recvdBodyLen = 0;
    while (recvdBodyLen < packet.request_header.request_size) {
        in = read(socket, packet.request_body, packet.request_header.request_size);
        if(in < 1) {
            printf("Failed receiving request body: expected %i got %i\n", packet.request_header.request_size, in);
            printf("request type: %i\n", packet.request_header.request_type);
            return;
        }
        recvdBodyLen+= in;
    }
    
    if(recvdBodyLen < packet.request_header.request_size) {
        printf("Body too short: expected %i got %i\n", header_len, recvdBodyLen);
        return;
    }

    switch (packet.request_header.request_type)
    {
    case REQUEST_TYPE_POSE_STREAM_INIT:
        requested_streams[((uint8_t*)(packet.request_body))[0]] = ((uint8_t*)(packet.request_body))[1];
        printf("Client requested stream for pose_class %i, object ID %i\n", ((uint8_t*)(packet.request_body))[0], ((uint8_t*)(packet.request_body))[1]);
        send_request_response(packet.request_header.request_type, packet.request_header.request_id, 0);
        break;
    
    default:
        printf("Unknown request type %i\n", packet.request_header.request_type);
        break;
    }
}

void PoseStreamerServerClient::handle_clock_request() {
    packetpreamble preamble;
    packetheader header;
    clockresponse response;

    header.header_size = 8;
    header.packet_type = PACKET_TYPE_CLOCK_RESPONSE;

    response.time = CurrentTime_nanoseconds();

    socket_mutex.lock();
    size_t out = 0;

    out = write(socket, &preamble, sizeof(preamble));
    if(out < sizeof(preamble)) m_valid=false;

    out = write(socket, &header, sizeof(header));
    if(out < sizeof(header)) m_valid=false;

    out = write(socket, &response, sizeof(response));
    if(out < sizeof(response)) m_valid=false;
    socket_mutex.unlock();
}

void PoseStreamerServerClient::publishStream(int pose_class, int obj_id, long timestamp, std::vector<double> pose) {
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
    stream_header.timestamp = timestamp;
    stream_header.blob_size = 8*pose.size();

    posestreamer::packetheader packet_header;
    packet_header.packet_type = PACKET_TYPE_STREAM;
    packet_header.header_size = sizeof(stream_header);

    packetpreamble preamble;

    socket_mutex.lock();

    size_t out;

    out = write(socket, &preamble, sizeof(preamble));
    if(out < sizeof(preamble)) m_valid=false;

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