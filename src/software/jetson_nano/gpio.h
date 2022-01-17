#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(GpioState, LOW, HIGH);
MAKE_ENUM(GpioDirection, INPUT, OUTPUT);

class GPIO
{
   public:
    /*
     * GPIO Sysfs Wrapper
     *
     * See https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
     *
     * @param gpio The gpio to setup
     * @param initial_direction The
     */
    GPIO(std::string gpio_number, GpioDirection initial_direction,
         GpioState initial_state);

    /**
     * Set the value
     *
     * @param state The state
     */
    void setValue(GpioState state);

    /**
     * Get the current value of the thing
     */
    GpioState getValue(void);

   private:
    std::string gpio_number_;
    GpioState current_state_;
    FILE *gpio_file_descriptor_;
    char buf_[1];
};
