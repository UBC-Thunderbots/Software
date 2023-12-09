#pragma once
#include <optional>
#include <string>
#include <filesystem>
#include <vector>

class ArduinoUtil
{
   public:
    /**
     * if an Arduino is connected, finds the port name of the device
     * @return device port in the form "/dev/tty*"
     */
    static std::optional<std::string> getArduinoPort();

   private:
    /**
     * internal struct used to pass around info from linux filesystem
     */
    struct HwInfo
    {
        std::string vendor;
        std::string product;
    };

    /**
     * goes through filesystem to get Vendor and Product info of a serial device
     * @param port the port associated to the device
     * @return Vendor and Product info
     */
    static std::optional<ArduinoUtil::HwInfo> getInfo(std::string port);

    /**
     * gets a list of all Serial devices in the system
     * @return A Vector of device names (ex "ttyACM0")
     */
    static std::vector<std::string> getSerialDevices();

    /**
     * reads the first line from a file
     * @param path path to the file
     * @return the first line in the file
     */
    static std::optional<std::string> readFileLine(std::filesystem::path path);
};
