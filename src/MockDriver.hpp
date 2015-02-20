#ifndef A4988_ARDUINO_DRIVER_HPP
#define A4988_ARDUINO_DRIVER_HPP

#include <chrono>
#include <iostream>

class MockDriver
{
public:
    MockDriver() :
            sleeping(false),
            enabled(true)
    {
        std::cout << "MockDriver()" << std::endl;
    }

    void writeReset()
    {
        std::cout << "writeReset() at " << clock() << " us" << std::endl;
        sleeping = false;
        enabled = true;
    }

    void writeNotSleep(uint8_t state)
    {
        sleeping = (state == 0);
        std::cout << "writeNotSleep(" << state << ") at " << clock() << " us" << std::endl;
    }

    uint8_t readNotSleep() const
    {
        return sleeping ? 1 : 0;
    }

    void writeNotEnable(uint8_t state)
    {
        enabled = (state == 0);
        std::cout << "writeNotEnable(" << state << ") at " << clock() << " us" << std::endl;
    }

    uint8_t readNotEnable() const
    {
        return enabled ? 1 : 0;
    }

    void writeForwardStep()
    {
        std::cout << "writeForwardStep() at " << clock() << " us" << std::endl;
    }

    void writeBackwardStep()
    {
        std::cout << "writeBackwardStep() at " << clock() << " us" << std::endl;
    }

    unsigned long int clock() const
    {
        auto n = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::time_point_cast<std::chrono::microseconds>(n);
        auto ticks = us.time_since_epoch().count();
        return static_cast<unsigned long>(ticks);
    }

private:
    bool sleeping;
    bool enabled;

};

#endif