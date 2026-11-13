#ifdef PLATFORMIO_BUILD
#include "esp_hal.h"
#include "power_board_runtime.h"
#else
#include "software/power/esp/esp_hal.h"
#include "software/power/power_board_runtime.h"
#endif

#include <Arduino.h>

#include <memory>
#include <vector>

#ifdef DEBUG_POWERLOOP
/**
 * This section of code is reserved for DIRECT testing on the microcontroller/debug
 * functionality.
 */
void setup() {}
void loop() {}
#else

std::shared_ptr<EspHal> power_hal;
std::shared_ptr<PowerBoardRuntime> runtime;

void setup()
{
    Serial.begin(460800, SERIAL_8N1);
    power_hal = std::make_shared<EspHal>();
    runtime   = std::make_shared<PowerBoardRuntime>(*power_hal);
}

void loop()
{
    while (Serial.available() > 0)
    {
        runtime->receiveByte(static_cast<uint8_t>(Serial.read()));
    }

    const std::vector<uint8_t> packet = runtime->tick();
    Serial.write(packet.data(), packet.size());
    Serial.flush();

    delay(5);
}
#endif
