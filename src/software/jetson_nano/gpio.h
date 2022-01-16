#pragma once
#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(GpioState, LOW, HIGH);

class GPIO
{
   public:
    /*
     * GPIO Sysfs Wrapper
     *
     * @param gpio The gpio to setup
     */
    GPIO(std::string gpio_number);
    virtual ~GPIO();

    /**
     * TODO
     */
    void setValue(GpioState state);

    /**
     * TODO
     */
    GpioState getValue(void);

   private:
    std::string gpio_number_;
    GpioState current_state_;
    FILE* gpio_file_descriptor_;
    char buf_[1];
};
