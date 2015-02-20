#include <iostream>
#include <chrono>
#include <thread>

#include "a4988.hpp"
#include "MockDriver.hpp"

using namespace std;

int main() {
    A4988 stepper(10.0f, 2.0f);
    cout << "stepper.position()     = " << stepper.position() << endl;
    cout << "stepper.acceleration() = " << stepper.acceleration() << endl;
    stepper.moveTo(100);
    //stepper.runAtVelocity(7.0f);
    bool stopped = false;
    while (stepper.poll())
    {
        cout << "stepper.position() = " << stepper.position() << ", stepper.velocity() = " << stepper.velocity() << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    cout << "stepper.position() = " << stepper.position() << ", stepper.velocity() = " << stepper.velocity() << endl;

    return 0;
}

