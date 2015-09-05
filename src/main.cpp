#include <stdio.h>

#include <avr/interrupt.h>

extern "C" {
#include "stream.h"
#include "uart.h"
#include "elapsed/elapsed.h"
}

#include "a4988.hpp"
#include "MockDriver.hpp"


int main() {
    init_millis();

    sei();

    stdin = stdout = get_uart0_stream();

    // USB Serial 0
    uart0_init(UART_BAUD_SELECT(115200, F_CPU));

    A4988<MockDriver> stepper(10.0f, 2.0f, 16, MILLISECONDS_PER_SECOND);
    //cout << "stepper.position()     = " << stepper.position() << endl;
    //cout << "stepper.acceleration() = " << stepper.acceleration() << endl;
    stepper.moveTo(100);
    //stepper.runAtVelocity(7.0f);
    bool stopped = false;
    while (stepper.poll())
    {
        //cout << "stepper.position() = " << stepper.position() << ", stepper.velocity() = " << stepper.velocity() << endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (stepper.position() == 90) {
            if (!stopped)
            {
                //stepper.stop();  // works
                //stepper.moveTo(5);
                stepper.runAtVelocity(-3.0f);
                stopped = true;
            }
            //stepper.moveTo(5);
        }
    }
    //cout << "stepper.position() = " << stepper.position() << ", stepper.velocity() = " << stepper.velocity() << endl;

    return 0;
}
