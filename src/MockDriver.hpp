#ifndef A4988_MOCK_DRIVER_HPP
#define A4988_MOCK_DRIVER_HPP

extern "C" {
#include "stream.h"
#include "uart.h"
#include "elapsed/elapsed.h"
}

class MockDriver
{
public:
    MockDriver() :
            sleeping(false),
            enabled(true)
    {
        printf("MockDriver()\n");
    }

    void writeReset()
    {
        printf("writeReset() at %lu ms\n", clock());
        sleeping = false;
        enabled = true;
    }

    void writeNotSleep(uint8_t state)
    {
        sleeping = (state == 0);
        printf("writeNotSleep(%d) at %lu ms\n", state, clock());
    }

    uint8_t readNotSleep() const
    {
        return sleeping ? 1 : 0;
    }

    void writeNotEnable(uint8_t state)
    {
        enabled = (state == 0);
        printf("writeNotEnable(%d) at %lu ms\n", state, clock());
    }

    uint8_t readNotEnable() const
    {
        return enabled ? 1 : 0;
    }

    void writeForwardStep()
    {
        printf("writeForwardStep() at %lu ms\n", clock());
    }

    void writeBackwardStep()
    {
        printf("writeBackwardStep() at %lu ms\n", clock());
    }

    unsigned long int clock() const
    {
        return millis();
    }

private:
    bool sleeping;
    bool enabled;

};

#endif