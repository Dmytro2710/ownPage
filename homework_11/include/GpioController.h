#pragma once
#include <gpiod.h>
#include <unistd.h>

class GpioController {
    public:
        GpioController(const char* chipname, int start_line_num, int drop_line_num);
        ~GpioController();
        void setStart(int value);
        void pulseDrop(); 
        bool isReady() const;
    private:
        gpiod_chip* chip_ = nullptr;
        gpiod_line* start_ = nullptr;
        gpiod_line* drop_ = nullptr;
};