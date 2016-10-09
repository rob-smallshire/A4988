#ifndef A4988_PORT_DRIVER_HPP
#define A4988_PORT_DRIVER_HPP

#include <stdio.h>

#include <avr/io.h>

#include <util/delay.h>

extern "C" {
#include "stream.h"
#include "uart.h"
#include "elapsed/elapsed.h"
}

#define sb(ADDRESS, BIT) (ADDRESS |= (1<<BIT))
#define cb(ADDRESS, BIT) (ADDRESS &= ~(1<<BIT))
#define fb(ADDRESS, BIT) (ADDRESS ^= (1<<BIT))
#define tb(ADDRESS, BIT) (ADDRESS & (1<<BIT))
#define xb(RADDRESS, RBIT, WADDRESS, WBIT) (tb(RADDRESS, RBIT) ? sb(WADDRESS, WBIT) : cb(WADDRESS, WBIT))


#define BED_NENABLE _BV(PORTA7)
#define BED_M1      _BV(PORTA6)
#define BED_M2      _BV(PORTA5)
#define BED_M3      _BV(PORTA4)
#define BED_NRESET  _BV(PORTA3)
#define BED_NSLEEP  _BV(PORTA2)
#define BED_STEP    _BV(PORTA1)
#define BED_DIR     _BV(PORTA0)

#define PULSE_DELAY_US 1

class PortDriver
{
public:
    PortDriver()
    {
        printf("PortDriver()\n");
        // Set all pins of PORTA for output)
        DDRA  = 0xff;
        writeReset();
        _delay_ms(500);
    }

    void writeReset()
    {
        printf("writeReset()\n");
        PORTA = BED_NSLEEP | BED_NRESET | BED_NENABLE;
        _delay_us(PULSE_DELAY_US);
        PORTA &= ~BED_NRESET;
        _delay_us(PULSE_DELAY_US);
        PORTA |= BED_NRESET;
    }

    void writeNotSleep(uint8_t state)
    {
        printf("writeNotSleep(%d) at %lu ms\n", state, clock());
        if (state)
        {
            PORTA |= BED_NSLEEP;
        }
        else
        {
            PORTA &= ~BED_NSLEEP;
        }
    }

    uint8_t readNotSleep() const
    {
        return PORTA & BED_NSLEEP;
    }

    void writeNotEnable(uint8_t state)
    {
        printf("writeNotEnable(%d) at %lu ms\n", state, clock());
        if (state)
        {
            PORTA |= BED_NENABLE;
        }
        else
        {
            PORTA &= ~BED_NENABLE;
        }
    }

    uint8_t readNotEnable() const
    {
        return PORTA & BED_NENABLE;
    }

    void writeForwardStep()
    {
        printf("writeForwardStep() at %lu ms\n", clock());
        if ((PORTA & BED_DIR) == 0)
        {
            PORTA |= BED_DIR;
            _delay_us(PULSE_DELAY_US);
        }
        PORTA |= BED_STEP;
        _delay_us(PULSE_DELAY_US);
        PORTA &= ~ BED_STEP;
        _delay_us(PULSE_DELAY_US);
    }

    void writeBackwardStep()
    {
        printf("writeBackwardStep() at %lu ms\n", clock());
        if ((PORTA & BED_DIR) != 0)
        {
            PORTA &= ~BED_DIR;
            _delay_us(PULSE_DELAY_US);
        }
        PORTA |= BED_STEP;
        _delay_us(PULSE_DELAY_US);
        PORTA &= ~ BED_STEP;
        _delay_us(PULSE_DELAY_US);
    }

    unsigned long int clock() const
    {
        return millis();
    }
};

#endif