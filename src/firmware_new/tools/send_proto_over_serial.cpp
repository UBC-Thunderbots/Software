/*
 * This tool sends protobuf over serial to the STM32 to do simple arithmatic. It serves
 * as a placeholder to demonstrate protobuf communication, as well as facilitate fw
 * development.
 *
 * Simply modify robot.proto in firmware_new/proto and run this tool to get access to the
 * updated protobuf fields.
 *
 * We are using proto2 (mostly to be consistent with the ssl proto version)
 * For more info on protobuf, please go here:
 * https://developers.google.com/protocol-buffers/docs/cpptutorial
 *
 * NOTE: We include robot.pb.h which contains the required info to create msgs
 * defined in robot.proto. We link the cc_library w/ c++ headers to this tool.
 * On the microcontroller side we also include robot.pb.h, but we link the
 * cc_library w/ c headers.
 *
 */

#include <libusb.h>

#include "firmware_new/proto/robot.pb.h"

using namespace std;
void printdev(libusb_device *dev);

int main()
{
    libusb_init(NULL);  // initialize the library for the session we just declared

    libusb_device_handle *dev_handle = libusb_open_device_with_vid_pid(NULL, 1155, 14158);
    libusb_device *device            = libusb_get_device(dev_handle);
    printdev(device);

    if (dev_handle == NULL)
        cout << "Cannot open device" << endl;
    else
        cout << "Device Opened" << endl;

    unsigned char *data = new unsigned char[4];  // data to write
    data[0]             = 'a';
    data[1]             = 'b';
    data[2]             = 'c';
    data[3]             = 'd';  // some dummy values

    int actual;  // used to find out how many bytes were written
    if (libusb_kernel_driver_active(dev_handle, 0) == 1)
    {  // find out if kernel driver is attached
        cout << "Kernel Driver Active" << endl;
        if (libusb_detach_kernel_driver(dev_handle, 0) == 0)  // detach it
            cout << "Kernel Driver Detached!" << endl;
    }
    int r = libusb_claim_interface(
        dev_handle, 0);  // claim interface 0 (the first) of device (mine had jsut 1)
    if (r < 0)
    {
        cout << "Cannot Claim Interface" << endl;
        return 1;
    }
    cout << "Claimed Interface" << endl;

    cout << "Data->" << data << "<-"
         << endl;  // just to see the data we want to write : abcd
    // cout<<"Writing Data..."<<endl;
    // r = libusb_bulk_transfer(dev_handle, (1 | LIBUSB_ENDPOINT_OUT), data, 4, &actual,
    // 0); //my device's out endpoint was 2, found with trial- the device had 2 endpoints:
    // 2 and 129 if(r == 0 && actual == 4) //we wrote the 4 bytes successfully
    // cout<<"Writing Successful!"<<endl;
    // else
    // cout<<"Write Error"<<endl;

    cout << "readign data....." << endl;

    r = libusb_bulk_transfer(dev_handle, (1 | LIBUSB_ENDPOINT_IN), data, 4, &actual,
                             0);  // my device's out endpoint was 2, found with trial- the
                                  // device had 2 endpoints: 2 and 129
    if (r == 0 && actual == 4)  // we wrote the 4 bytes successfully
        cout << "Reading Successful!" << endl;
    else
        cout << "Read Error" << endl;

    r = libusb_release_interface(dev_handle, 0);  // release the claimed interface
    if (r != 0)
    {
        cout << "Cannot Release Interface" << endl;
        return 1;
    }
    cout << "Released Interface" << endl;

    libusb_close(dev_handle);  // close the device we opened

    delete[] data;  // delete the allocated memory for data
    return 0;
}
void printdev(libusb_device *dev)
{
    libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
    {
        cout << "failed to get device descriptor" << endl;
        return;
    }
    cout << "Number of possible configurations: " << (int)desc.bNumConfigurations << "  ";
    cout << "Device Class: " << (int)desc.bDeviceClass << "  ";
    cout << "VendorID: " << desc.idVendor << "  ";
    cout << "ProductID: " << desc.idProduct << endl;
    libusb_config_descriptor *config;
    libusb_get_config_descriptor(dev, 0, &config);
    cout << "Interfaces: " << (int)config->bNumInterfaces << " ||| ";
    const libusb_interface *inter;
    const libusb_interface_descriptor *interdesc;
    const libusb_endpoint_descriptor *epdesc;
    for (int i = 0; i < (int)config->bNumInterfaces; i++)
    {
        inter = &config->interface[i];
        cout << "Number of alternate settings: " << inter->num_altsetting << " | ";
        for (int j = 0; j < inter->num_altsetting; j++)
        {
            interdesc = &inter->altsetting[j];
            cout << "Interface Number: " << (int)interdesc->bInterfaceNumber << " | ";
            cout << "Number of endpoints: " << (int)interdesc->bNumEndpoints << " | ";
            for (int k = 0; k < (int)interdesc->bNumEndpoints; k++)
            {
                epdesc = &interdesc->endpoint[k];
                cout << "Descriptor Type: " << (int)epdesc->bDescriptorType << " | ";
                cout << "EP Address: " << (int)epdesc->bEndpointAddress << " | ";
            }
        }
    }
    cout << endl << endl << endl;
    libusb_free_config_descriptor(config);
}
