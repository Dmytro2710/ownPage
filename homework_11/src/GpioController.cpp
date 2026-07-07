#include "GpioController.h"
#include "Logger.h"
#include <unistd.h>

#define PULSE_DURATION_US 500000

GpioController::GpioController(const char* chipname, int start_line_num, int drop_line_num) {
    chip_ = gpiod_chip_open_by_name(chipname);
    if (!chip_) { LOG("Failed to open GPIO chip"); return; }
    start_ = gpiod_chip_get_line(chip_, start_line_num);
    if (!start_) { LOG("Failed to get start line"); return; }
    drop_ = gpiod_chip_get_line(chip_, drop_line_num);
    if (!drop_) { LOG("Failed to get drop line"); return; }
    if (gpiod_line_request_output(start_, "drone", 0) < 0) {
        LOG("Failed to request start line as output");
        return;
    }
    if (gpiod_line_request_output(drop_, "drone", 0) < 0) {
        LOG("Failed to request drop line as output");
        return;
    }
}

void GpioController::setStart(int value) {
    if (gpiod_line_set_value(start_, value) < 0) {
        LOG("Failed to set start line value");
    }
}

void GpioController::pulseDrop() {
    LOG("pulseDrop: setting HIGH on line " << gpiod_line_offset(drop_));
    if (gpiod_line_set_value(drop_, 1) < 0) {
        LOG("Failed to set drop line value to 1");
        return;
    }
    LOG("pulseDrop: sleeping " << PULSE_DURATION_US << "us");
    usleep(PULSE_DURATION_US);
    LOG("pulseDrop: setting LOW on line " << gpiod_line_offset(drop_));
    if (gpiod_line_set_value(drop_, 0) < 0) {
        LOG("Failed to set drop line value to 0");
    }
    LOG("pulseDrop: done");
}

GpioController::~GpioController() {
    if (start_) gpiod_line_release(start_);
    if (drop_) gpiod_line_release(drop_);
    if (chip_) gpiod_chip_close(chip_);
    start_ = nullptr;
    drop_ = nullptr;
    chip_ = nullptr;
}

bool GpioController::isReady() const {
    return chip_ && start_ && drop_;
}