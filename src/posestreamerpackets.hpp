#include <stdlib.h>

namespace posestreamer
{
    enum  {
        PACKET_TYPE_REQUEST = 0x01,
        PACKET_TYPE_REQUEST_RESPONSE = 0x02,
        PACKET_TYPE_STREAM = 0x10,

        REQUEST_TYPE_IDENTIFY = 0x01,
        REQUEST_TYPE_POSE_STREAM_INIT = 0x10
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

    struct requestpacket
    {
        requestheader request_header;
        void *request_body;
    };

    struct streamheader 
    {
        char pose_class;
        char object_id;
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



