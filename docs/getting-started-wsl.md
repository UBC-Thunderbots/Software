# Software Setup on Windows Subsystem for Linux

## Table of Contents

<!--TOC-->

- [Software Setup on Windows Subsystem for Linux](#software-setup-on-windows-subsystem-for-linux)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [WSL2 Setup (Windows 11 - Recommended)](#wsl2-setup-windows-11---recommended)
  - [WSLg Setup (Windows 11/10 - Recommended)](#wslg-setup-windows-1110---recommended)
  - [WSL2 Setup (Windows 10)](#wsl2-setup-windows-10)
    - [X Server Setup](#x-server-setup)
  - [Networking Issues](#networking-issues)
  - [USB Issues](#usb-issues)

<!--TOC-->

## Introduction

Windows has a Windows Subsystem for Linux component that can be used to develop and run code for Linux on Windows. WSL1 was a Windows component that implemented Linux kernel interfaces, and didn't work great with Thunderbots software. WSL2 runs a full-fledged Linux kernel in a VM, and works great with Thunderbots software with the exception that we need to use software rendering instead of GPU-accelerated rendering for our AI. WSLg is WSL2 but with built-in support for running GUI applications (e.g. Thunderscope).

Update for 2025: WSL2 now comes prepackaged with support for running GUI applications like Thunderscope. Setup is now significantly simplified.

> [!WARNING]  
> **Support for WSL is experimental. Performance will be degraded, features may not work properly, and the developer experience will be worse overall.**

## WSL2 Setup (Windows 11 - Recommended)
Installing WSL2 through Windows 11 is very simple. \
[Reference](https://documentation.ubuntu.com/wsl/latest/howto/install-ubuntu-wsl2/)
1. First, we enable WSL2: \
Open PowerShell and run: `wsl --install`. You may be required to grant administrator permissions and restart your computer.\
2. Then, install Ubuntu 24.04. Type `wsl --list --online` to list the available distributions, and look for the Ubuntu 24.04 LTS entry in the Friendly name column.
![alt text](/images/wsldistros.png)
3. Follow [Software Setup](./getting-started.md).


## WSLg Setup (Windows 11/10 - Recommended)
1. Installing WSLg is more straight forward than WSL2 and is the recommended way to run Linux GUI applications in Windows. For up to date documentation, please follow the [official documentation for setting up WSLg](https://github.com/microsoft/wslg#installing-wslg). 
2. Once you have completed all of the above, complete the [Software Setup](./getting-started.md).


## WSL2 Setup (Windows 10)
If you are not using Windows 11 or the latest version of Windows 10 and would prefer not to upgrade, you can follow the following steps. Note that this setup is more complex than the [WSLg](#wslg-setup-(windows-11---recommended)) setup as it does not support GUI applications out of the box.
1. You'll need to be on build 19041 or later to use WSL2. If you have updated to Windows 10 version 2004 or newer, you will be able to use WSL2. 
2. When you have ensured that your Windows version supports WSL2, do the following to enable it.
    - Enable WSL by opening an Administrator PowerShell window and running command 
        ```
        dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
        ```
    - Enable the 'Virtual Machine Platform' component by Administrator PowerShell window and running command 
        ```
        dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
        ``` 
    - Reboot your machine.
3. Now, let's install Ubuntu.
    - Download the WSL2 kernel from [here](https://docs.microsoft.com/en-us/windows/wsl/wsl2-kernel).
    - Open a PowerShell window and run command `wsl --set-default-version 2` to use WSL2 by default.
    - Install Ubuntu 22.04 LTS or Ubuntu 24.04 LTS from the Microsoft Store.
    - Open the Ubuntu app in the Start menu. It will open a command prompt and ask you to create a new UNIX username and password for your WSL2 Ubuntu installation. 

### X Server Setup
1. [Download and install VcXsrv](https://sourceforge.net/projects/vcxsrv/files/latest/download)
2. Start WSL and use your terminal text editor of choice to edit `/etc/profile.d/wsl-integration.sh` and change the line `export LIBGL_ALWAYS_INDIRECT=1` to `# export LIBGL_ALWAYS_INDIRECT=1`
    - This comments out the line that sets the `LIBGL_ALWAYS_INDIRECT` environment variable which sets OpenGL programs to render on the X server instead of directly using the local OpenGL drivers. Unfortunately, this only supports OpenGL 1.4 which makes thunderbots give a black window.
    - This makes WSL use the `llvmpipe` software rasterizer instead, which will run extremely poorly on low spec machines. 
3. Create a firewall exception for VcXsrv. This is required because the X Window System communication between the Hyper-V Linux VM and the Windows-side X server appears as network traffic over the `vEthernet (WSL)` network interface from another computer.
    - Open the Start menu and type "Firewall & network protection" to take you to the firewall settings.
    - Click "Allow an app through firewall". A window will appear titled "Allow apps to communicate through Windows Defender firewall".
    - Click "Allow another app" on the window that just popped up. Hit "Browse" and navigate to where you installed "VcXsrv", usually `C:\Program Files\VcXsrv\`. Select `VcXsrv.exe`.
    - Click "Network types..." in the bottom-left corner. Check off `Private` and `Public`.
    - Click add.
4. Start an X server by opening Xlaunch through the Start menu.
    - Choose 'Multiple windows' on the first screen
    - Choose 'Start no client' on the second screen
    - On the third screen, make sure 'Native opengl' is UNCHECKED and 'Disable access control' is CHECKED
5. Shut down WSL with `wsl --shutdown` on the Windows command line. Open WSL again through the Ubuntu app in the Start menu.
6. Install glxgears by updating your package lists with `sudo apt update`, and then installing with `sudo apt install mesa-utils`. glxgears is a tool we will use to validate that everything is set up correctly.
7. Verify that your system is configured correctly by running `glxgears -info` on the Linux command line. You should see a window pop up with spinning gears and the line `GL_RENDERER   = llvmpipe (LLVM 9.0, 256 bits)` at the top of the output.

Once you have completed all of the above, complete the [Software Setup](./getting-started.md).

## Networking Issues

Networking compatibility with WSL is limited but it can be improved by enabling [mirrored mode](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking). There are still many unresolved issues with mirrored mode enabled (no vision, no robot status, etc.) but robot diagnostics should work and you should be able to control robots on the network.

Create a `.wslconfig` file in your `%UserProfile%` directory (typically your home directory, `cd ~`) and copy the following into the config file:

```
[wsl2]
networkingMode = mirrored
```

This will enable [mirrored mode networking for WSL](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking). This mode “mirrors” the networking interfaces you have on Windows onto Linux, which improves networking capabilities and compatibility.

When selecting a network interface to use, choose `eth...`/`en...` or similar. There probably won’t be a `wlan` interface since WSL only sees the virtual network interface `eth...`. 

## USB Issues

WSL does not natively support connecting USB devices, which is necessary for some tasks like flashing firmware onto our robots or using a physical e-stop. You will need to install a piece of open-source software called `usbipd-win` to support USB connectivity.

Please follow the [official documentation on installing `usbipd-win`](https://github.com/dorssel/usbipd-win?tab=readme-ov-file#how-to-install).

Note that connected devices are not automatically shared with `usbipd`, so you will have to manually share the device with `usbipd` and attach/detach the device to a `usbipd` client whenever you plug/unplug it (or between reboots). See the [official documentation for details on usage](https://github.com/dorssel/usbipd-win?tab=readme-ov-file#how-to-install).
