#include <pigpio.h>
#include <iostream>

int main() {
    // Initialize pigpio library
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio library" << std::endl;
        return 1;
    }

    while (1)
    {
        /* code */
    
    
        // Set pin 40 as an output
        gpioSetMode(21, PI_OUTPUT);

        // Turn on the LED (drive pin 40 high)
        gpioWrite(21, 1);

        // Wait for a while
        time_sleep(1);  // Sleep for 5 seconds

        // Turn off the LED (drive pin 40 low)
        gpioWrite(21, 0);

        // Wait for a while
        time_sleep(1);  // Sleep for 5 seconds
    }

    // Release resources
    gpioTerminate();

    return 0;
}