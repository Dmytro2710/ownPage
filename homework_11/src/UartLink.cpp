#include "drone_link.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include "UartLink.h"
#include <fstream>
#include "Logger.h"

using namespace dlink;


UartLink::UartLink(const char* device) {
    fd_ = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) { perror("open"); return; }
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    tio.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tio);
}

bool UartLink::readPacket(uint8_t& type, uint8_t* payload, uint8_t& len) {
    while (pending_start_ < pending_end_) {
        uint8_t b = pending_[pending_start_++];
        if (parser_.feed(b, type, payload, len))
            return true;
    }
    pending_start_ = 0;
    pending_end_   = 0;
    int n = read(fd_, pending_, sizeof(pending_));
    if (n <= 0) {
        //LOG("read returned " << n);
        return false;
    }
    //LOG("read got " << n << " bytes");
    pending_end_ = n;

    while (pending_start_ < pending_end_) {
        uint8_t b = pending_[pending_start_++];
        if (parser_.feed(b, type, payload, len))
            return true;
    }
    return false;
}

void UartLink::sendControl(float accel, float turnRate) {
    uint8_t out[64];
    dlink::Control c{accel, turnRate};
    size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof(c), out);
    
    /*printf("[TX] ");
    for (size_t i = 0; i < m; i++) printf("%02X ", out[i]);
    printf("\n");
    fflush(stdout);*/
    
    write(fd_, out, m);
}

bool UartLink::isOpen() const { return fd_ >= 0; }

UartLink::~UartLink() {
    if (fd_ >= 0) close(fd_);
}

