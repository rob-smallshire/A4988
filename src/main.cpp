#include <stdio.h>

#include <avr/interrupt.h>

extern "C" {
#include "stream.h"
#include "uart.h"
#include "elapsed/elapsed.h"
}

#include "a4988.hpp"
#include "MockDriver.hpp"
#include "PortDriver.hpp"

int main() {
    init_millis();

    sei();

    stdin = stdout = get_uart0_stream();

    // USB Serial
    uart0_init(UART_BAUD_SELECT(115200, F_CPU));

    A4988<PortDriver> stepper(400.0f, 20.0f, 16, 1000.0f);
    stepper.setEnabled(true);
    //cout << "stepper.position()     = " << stepper.position() << endl;
    //cout << "stepper.acceleration() = " << stepper.acceleration() << endl;
    stepper.moveTo(384);
    //stepper.runAtVelocity(7.0f);
//    bool stopped = false;
    while (stepper.poll())
    {
        //cout << "stepper.position() = " << stepper.position() << ", stepper.velocity() = " << stepper.velocity() << endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        if (stepper.position() == 90) {
//            if (!stopped)
//            {
//                //stepper.stop();  // works
//                //stepper.moveTo(5);
//                stepper.runAtVelocity(-3.0f);
//                stopped = true;
//            }
//            //stepper.moveTo(5);
//        }
    }
    _delay_ms(5000);
    stepper.setEnabled(false);
    //cout << "stepper.position() = " << stepper.position() << ", stepper.velocity() = " << stepper.velocity() << endl;
    while (true);
    return 0;
}
