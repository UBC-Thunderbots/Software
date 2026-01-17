# Raspberry Pi Setup Guide

## Install Raspberry Pi OS onto MicroSD card

### Introduction

The microSD card in a Raspberry Pi holds its Operating System and acts as its primary storage area. The Pi will not work without it.

The Raspberry Pi 5 is compatible with many different (primarily Linux-based) operating systems. The one we use is the standard Raspberry Pi OS (also Linux-based). 

### Install OS Using Raspberry Pi Imager

To install the Raspberry Pi OS onto a microSD card, we use the Raspberry Pi Imager program. Install it [here](https://www.raspberrypi.com/software/). 

When you open the imager, you should see something like this:

<img width="402" height="159" alt="image" src="https://github.com/user-attachments/assets/b31e03e3-0d25-4b99-b0d5-0168387f715d" />

Once installed, follow these steps to set up the Raspberry Pi:
1. Plug the microSD card into your device.
    * If your device has an SD card slot, use a microSD to SD card adapter. This adapter is available in the tbots EDC space as of Jan 2026. 
    * If your device only has a USB slot, use a combination of a microSD to SD card adapter and USB SD card reader. Again, both of these adapters are available in the tbots EDC space.
2. Open and configure the Raspberry Pi Imager program as follows:
    * Raspberry Pi Device: Raspberry Pi 5
    * Operating System: Raspberry Pi OS (64-bit)
    * Storage: `<your_inserted_micro_SD_card>`
    * Customization:
      * Hostname: `<robot_name>`.local (Ex: `aimbot.local`) 
      * Localization -> Timezone: America/Vancouver
      * User -> Username: `<robot_name>` (Ex: `aimbot.local`)
      * User -> Password: `<password>` (ask someone if you don't know what the correct password is)
      * Wifi -> Secure Network -> SSID: `<tbots_wifi_name>` (`tbots` as of Jan 2026)
      * Wifi -> Password: `<password>` (ask someone if you don't know what the correct password is)
      * Remote Access: Enable SSH and select "Use Password Authentication"
    * Click the "Write" button and wait until complete
  
  Once the write process is complete, the Raspberry Pi OS is set up! You can verify the write was successful using several methods; the following is one of them: 
  * Connect the Raspberry Pi to an HDMI output screen using a microHDMI to HDMI cable.
    * You can also use an HDMI to HDMI cable with a microHDMI to HDMI adapter. We have one of these adapters in the tbots EDC space as of Jan 2026.
  * The output screen on the HDMI should appear similar to a default laptop/PC screen. If this is the case, the write was successful.
  * If you do not see the above, and instead see text on a black screen with messages akin to "Unable to read partition as FAT" - the write was not successful.


