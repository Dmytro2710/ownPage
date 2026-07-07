#pragma once
#include "drone_link.h"
#include <string>

class UartLink {
public:
    UartLink(const char* device);
    ~UartLink();

    bool isOpen() const;
    bool readPacket(uint8_t& type, uint8_t* payload, uint8_t& len);
    void sendControl(float accel, float turnRate);

private:
    int fd_ = -1;
    dlink::Parser parser_;
    uint8_t pending_[4096];
    int pending_start_ = 0;
    int pending_end_   = 0;
};