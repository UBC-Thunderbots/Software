
#include "arduino_util.h"

#include <stdlib.h>
#include <unistd.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <iostream>

#include "shared/constants.h"


std::optional<std::string> ArduinoUtil::getArduinoPort()
{
    auto devices = getSerialDevices();

    for (auto device : devices)
    {
        std::optional<HwInfo> hwInfo = getInfo(device);
        if (hwInfo.has_value())
        {
            if (hwInfo.value().vendor == ARDUINO_VENDOR_ID &&
                hwInfo.value().product == ARDUINO_PRODUCT_ID)
            {
                std::string device_path = (boost::format("/dev/%1%") % device).str();
                return device_path;
            }
        }
    }

    return std::nullopt;
}

std::optional<ArduinoUtil::HwInfo> ArduinoUtil::getInfo(std::string port)
{
    // ported from
    // https://github.com/pyserial/pyserial/blob/bce419352b22b2605df6c2158f3e20a15b8061cb/serial/tools/list_ports_linux.py

    std::string device_path_string =
        (boost::format("/sys/class/tty/%1%/device") % port).str();

    if (!boost::filesystem::is_directory(device_path_string))
    {
        return std::nullopt;
    }

    boost::filesystem::path device_path = realpath(device_path_string.c_str(), NULL);
    boost::filesystem::path subsystem;
    boost::filesystem::path usb_interface_path;
    boost::filesystem::path usb_device_path;

    if (!boost::filesystem::is_directory(device_path / "subsystem"))
    {
        return std::nullopt;
    }

    subsystem =
        boost::filesystem::basename(realpath((device_path / "subsystem").c_str(), NULL));

    if (subsystem == "usb-serial")
    {
        usb_interface_path = device_path.parent_path();
    }
    else if (subsystem == "usb")
    {
        usb_interface_path = device_path;
    }
    else
    {
        return std::nullopt;
    }

    if (!usb_interface_path.empty())
    {
        usb_device_path                    = usb_interface_path.parent_path();
        std::optional<std::string> vendor  = readFileLine(usb_device_path / "idVendor");
        std::optional<std::string> product = readFileLine(usb_device_path / "idProduct");

        if (vendor.has_value() && product.has_value())
        {
            HwInfo hwInfo({vendor.value(), product.value()});
            return hwInfo;
        }
    }

    return std::nullopt;
}

std::vector<std::string> ArduinoUtil::getSerialDevices()
{
    boost::filesystem::path dev("/dev");
    boost::filesystem::directory_iterator end_itr;
    std::vector<std::string> devices;

    for (boost::filesystem::directory_iterator itr(dev); itr != end_itr; ++itr)
    {
        std::string filename = itr->path().stem().string();
        if (filename.find("ttyUSB") != std::string::npos ||
            filename.find("ttyACM") != std::string::npos ||
            filename.find("ttyS") != std::string::npos)
        {
            devices.push_back(filename);
        }
    }

    return devices;
}

std::optional<std::string> ArduinoUtil::readFileLine(boost::filesystem::path path)
{
    boost::filesystem::ifstream f(path.c_str());
    std::string res;
    if (f.is_open())
    {
        std::getline(f, res);
        f.close();
        return res;
    }

    return std::nullopt;
}
