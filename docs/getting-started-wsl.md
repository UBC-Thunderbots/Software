# Software Setup on Windows Subsystem for Linux

## Table of Contents

* [Table Of Contents](#table-of-contents)
* [Introduction](#introduction)
* [WSLg Setup (Windows 11 - Recommended)](#wslg-setup-(windows-11---recommended))
* [WSL2 Setup (windows 10)](#wsl2-setup-(windows-10))
    * [X Server Setup](#x-server-setup)


## Introduction

Windows has a Windows Subsystem for Linux component that can be used to develop and run code for Linux on Windows. WSL1 was a Windows component that implemented Linux kernel interfaces, and didn't work great with Thunderbots software. WSL2 runs a full-fledged Linux kernel in a VM, and works great with Thunderbots software with the exception that we need to use software rendering instead of GPU-accelerated rendering for our AI. WSLg is WSL2 but with built-in support for running GUI applications (e.g. Thunderscope).

**Support for WSL is experimental. Because we use software rendering, the experience will also be degraded on computers with weak or old CPUs.**

**Note that this will not work with legacy robots. Due to the lack of USB support in WSL2, we are unable to use the USB dongle used to communicate with them.**

## WSLg Setup (Windows 11 - Recommended)
1. Installing WSLg is more straight forward than WSL2. For up to date documentation, please follow the [official documentation for setting up WSLg](https://github.com/microsoft/wslg#installing-wslg). 
2. Once you have completed all of the above, complete the [Software Setup](./getting-started.md).


## WSL2 Setup (Windows 10)
If you are not using Windows 11 and would prefer not to upgrade, you can follow the following steps. Note that this setup is more complex than the [WSLg](#wslg-setup-(windows-11---recommended)) setup as it does not support GUI applications out of the box.
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
    - Install Ubuntu 20.04 LTS or Ubuntu 22.04 LTS from the Microsoft Store.
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
