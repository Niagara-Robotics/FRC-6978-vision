#include <stdlib.h>

namespace posestreamer
{
    #define PACKET_TYPE_REQUEST 0x01
    #define PACKET_TYPE_REQUEST_RESPONSE 0x02
    #define PACKET_TYPE_STREAM 0x10

    #define REQUEST_TYPE_IDENTIFY 0x01
    #define REQUEST_TYPE_POSE_STREAM_INIT 0x10

    struct packetheader 
    {
        char packet_type;
        char header_size;
    };

    struct requestheader 
    {
        char request_type;
        char request_id;
        size_t request_size;
    };

    struct requestpacket
    {
        packetheader packet_header;
        requestheader request_header;
        char *request_body;
    };

    struct streamheader 
    {
        char pose_class;
        char object_id;
        size_t blob_size;
    };

    struct streampacket
    {
        packetheader packet_header;
        streamheader stream_header;
        char *blob;
    };

    struct streamrequest
    {
        char pose_class;
        char object_id;
    };
} // namespace posestreamer



