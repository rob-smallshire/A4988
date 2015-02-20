#ifndef A4988_ARDUINO_DRIVER_HPP
#define A4988_ARDUINO_DRIVER_HPP

template <uint8_t sleep_pin,
          uint8_t enable_pin,
          uint8_t direction_pin,
          uint8_t step_pin,
          uint8_t ms1_pin,
          uint8_t ms2_pin,
          uint8_t ms3_pin>
class ArduinoPinDriver
{
public:
    ArduinoPinDriver()
    {
    }

    void writeReset()
    {
    }

    void writeNotSleep(uint8_t state)
    {

    }

    uint8_t readNotSleep()
    {
        return 1;
    }

    void writeNotEnable(uint8_t state)
    {

    }

    uint8_t readNotEnable()
    {
        return 0;
    }

    void writeForwardStep()
    {

    }

    void writeBackwardStep()
    {

    }

    unsigned long int clock()
    {
        return 1000UL;
    }


};

#endif