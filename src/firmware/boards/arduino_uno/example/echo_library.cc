
#ifdef PLATFORMIO_BUILD
#include "echo_library.h" // PlatformIO sees and includes the library based on the file name ONLY
#else
#include "echo_library.h" // actual path to file (in this example it is the same)
#endif

int incomingByte = 0;

void echo() {
    // send data only when you receive data:
    if (Serial.available() > 0) {

        // read the incoming byte:
        incomingByte = Serial.read();

        // say what you got:
        Serial.print((char)incomingByte);
    }
}
