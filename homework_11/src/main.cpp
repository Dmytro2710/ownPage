#include "Types.h"
#include "Logger.h"
#include "MissionProcessor.h"
#include "GpioController.h"
#include "UartLink.h"
#include <csignal>
#include <memory>
#include <atomic>

std::atomic<bool> g_stop{false};

void signalHandler(int sig) {
    g_stop.store(true);
}

int main() {
    GpioController gpio("gpiochip0", 24, 23);
    UartLink uart("/dev/ttyAMA3");

    if (!gpio.isReady()) { LOG("GPIO not ready"); return 1; }
    if (!uart.isOpen())  { LOG("UART not open");  return 1; }

    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    auto mission = std::make_unique<MissionProcessor>(uart, gpio, g_stop);
    mission->run();

    gpio.setStart(0);
    return 0;
}