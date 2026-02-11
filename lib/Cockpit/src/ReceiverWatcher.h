class ReceiverWatcher {
public:
    virtual ~ReceiverWatcher() = default;
    virtual void new_receiver_packet_available() = 0;
};
