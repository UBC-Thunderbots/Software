# Raspberry Pi Setup Guide

## Install Raspberry Pi OS onto MicroSD card

### Introduction

The microSD card in a Raspberry Pi holds its Operating System and acts as its primary storage area. The Pi will not work without it.

The Raspberry Pi 5 is compatible with many different (primarily Linux-based) operating systems. The one we use is the standard Raspberry Pi OS (also Linux-based). 

### Install OS Using Raspberry Pi Imager

To install the Raspberry Pi OS onto a microSD card, we use the Raspberry Pi Imager program. Install it [here](https://www.raspberrypi.com/software/). 

When you open the imager, you should see something like this:

* <img width="402" height="159" alt="image" src="https://github.com/user-attachments/assets/b31e03e3-0d25-4b99-b0d5-0168387f715d" />

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
      * WiFi -> Secure Network -> SSID: `<tbots_wifi_name>` (`tbots` as of Jan 2026)
      * WiFi -> Password: `<password>` (ask someone if you don't know what the correct password is)
      * Remote Access: Enable SSH and select "Use Password Authentication"
    * Click the "Write" button and wait until complete
  
  Once the write process is complete, the Raspberry Pi OS is set up! You can verify the write was successful using several methods; the following is only one of these methods: 
  * Connect the Raspberry Pi to an HDMI output screen using a microHDMI to HDMI cable.
    * You can also use an HDMI to HDMI cable with a microHDMI to HDMI adapter. We have one of these adapters in the tbots EDC space as of Jan 2026.
  * The output screen on the HDMI should appear similar to a default laptop/PC screen. If this is the case, the write was successful.
  * If you do not see the above, and instead see text on a black screen with messages akin to "Unable to read partition as FAT" - the write was not successful.

## Configure Raspberry Pi with Thunderbots Service

### Introduction

We use the Raspberry Pi OS, which is based on the Linux kernel. The Linux kernel uses systemd as its service manager. Our core service is [thunderloop.service](https://github.com/UBC-Thunderbots/Software/blob/master/src/software/embedded/linux_configs/systemd/thunderloop.service).

To run and manage the thunderloop service on the Pi, however, there are many dependencies (drivers, admin control, internet, etc.) that must be set up first. Luckily, all of this setup can be done using one command with an Ansible playbook. 

Read more about Ansible in our robot software architecture documentation [here](https://github.com/UBC-Thunderbots/Software/blob/master/docs/robot-software-architecture.md).

### Configuration

#### Internet Configuration

Before running the Ansible command, we must do the following to configure internet access to the Pi:
1. Ensure your Raspberry Pi is connected to internet through an ethernet cable. Ensure your ethernet cable is not broken, as there are many non-functioning ones in the Thunderbots EDC space.
2. Configure network settings through the following steps:
   1. Go to network settings. You should see something like this:
   
   * <img width="446" height="77.5" alt="image" src="https://github.com/user-attachments/assets/20f9b3e5-e629-40c7-ac49-024f34df66ca" />
   
   2. Click the settings icon on the right (seen in the image above).
   3. Under the IPv4 tab, select "Shared to other computers"
   
   * <img width="471.5" height="132.5" alt="image" src="https://github.com/user-attachments/assets/ee776fbc-7fec-4e7d-89da-35b48de4df4f" />
   
   4. Under the IPv6 tab, select "Disable"
   *  <img width="471.5" height="132.5" alt="image" src="https://github.com/user-attachments/assets/7ad2cba4-42bf-4204-a182-a087e7f632e2" />
   
   5. Apply changes
3. Enable IP forwarding from your device to the Pi with the following command (this has to be reconfigured every time you boot your device):
```bash 
sudo sysctl -w net.ipv4.ip_forward=1
```
4. Enable Network Address Translation (NAT) between your Pi and device with the following commands (this has to be reconfigured every time you boot your device):
```bash
sudo iptables -t nat -A POSTROUTING -o wlo1 -j MASQUERADE
sudo iptables -A FORWARD -i eno2 -o wlo1 -j ACCEPT
sudo iptables -A FORWARD -i wlo1 -o eno2 -m state --state RELATED,ESTABLISHED -j ACCEPT
```

#### Thunderloop Service Configuration Through Ansible
1. SSH into the Raspberry Pi from your device by connecting to the `tbots` WiFi and typing the following in the command terminal:
   * ssh `<robot_name>` (Ex: `ssh aimbot.local`)
   * enter the password when prompted
2. Verify that the Pi has internet connection by pinging Google's Public DNS Service with the following command:
```bash
ping -c 3 8.8.8.8
```
   * If successful, you should see all packets transmitted and received at some point in the return message. Ex: ```3 packets transmitted, 3 received, 0% packet loss, time 2004ms```.
   * If not successful, you will see packets not received. Ex: ```3 packets transmitted, 0 received, 100% packet loss, time 2052ms```.
3. Run the bazel ansible command:
```bash
bazel run //software/embedded/ansible:run_ansible --platforms=//toolchains/cc:robot  --//software/embedded:host_platform=PI -- --playbook setup_pi.yml --hosts aimbot.local --ssh_pass thunderbots
```
   * This may take a while.
4. Done!

### Common Errors and Debugging

**Cannot SSH into Pi** 
A: Confirm that your device is connected to the `tbots` WiFi

**Raspberry Pi shuts off (light turns red, HDMI output disconnects if connected) while the Bazel Ansible command is running**
A: This is a power brownout (voltage drops below required threshold). There are too many peripherals connected. Disconnect some and try again.

**Raspberry Pi unresponsive and LED always solid green**
A: This is usually an indicator that the Pi's SD Card is corrupted or empty. Operational Raspberry Pi's usually have a flickering LED. Fix by reprovisioning the SD Card with the Raspberry Pi Imager (directions above). 







