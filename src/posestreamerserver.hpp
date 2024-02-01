class PoseStreamerServer
{
private:
    void process_packet(int sock);
    void parse_request(int sock);
    
public:
    PoseStreamerServer(/* args */);
    ~PoseStreamerServer();
};


