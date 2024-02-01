#include "posestreamerpackets.hpp"
#include "posestreamerserver.hpp"

#include <stdio.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <istream>



void PoseStreamerServer::process_packet(int sock) 
{
    int packet_type;
    read(sock, &packet_type, 1);

    switch (packet_type)
    {
    case PACKET_TYPE_REQUEST:
        
        break;
    
    default:
        break;
    }
}

PoseStreamerServer::PoseStreamerServer()
{
}

PoseStreamerServer::~PoseStreamerServer()
{
}