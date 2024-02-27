#include <stdlib.h>

namespace posestreamer
{
    enum  {
        PACKET_PREAMBLE_1 = 0x05,
        PACKET_PREAMBLE_2 = 0xb3,

        PACKET_TYPE_REQUEST = 0x01,
        PACKET_TYPE_REQUEST_RESPONSE = 0x02,
        PACKET_TYPE_STREAM = 0x10,
        PACKET_TYPE_CLOCK_REQUEST = 0x31,
        PACKET_TYPE_CLOCK_RESPONSE = 0x30,

        REQUEST_TYPE_IDENTIFY = 0x01,
        REQUEST_TYPE_POSE_STREAM_INIT = 0x10
    };

    struct packetpreamble
    {
        char preamble_1 = PACKET_PREAMBLE_1;
        char preamble_2 = PACKET_PREAMBLE_2;
    };

    struct packetheader 
    {
        char packet_type;
        char header_size;
    };

    struct requestheader 
    {
        char request_type;
        char request_id;
        char request_size;
    };

    struct requestresponse 
    {
        char request_type;
        char request_id;
        char status_code;
    };

    struct clockresponse
    {
        long time;
    };

    struct requestpacket
    {
        requestheader request_header;
        void *request_body;
    };

    struct streamheader 
    {
        char pose_class;
        char object_id;
        long timestamp;
        char blob_size;
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



