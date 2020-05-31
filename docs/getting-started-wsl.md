# Software Setup on Windows Subsystem for Linux

## Table of Contents

* [Table Of Contents](#table-of-contents)
* [Introduction](#introduction)
* [WSL2 Setup](#wsl2-setup)
* [X Server Setup](#wsl2-setup)


## Introduction

Windows has a new Windows Subsystem for Linux component that can be used to develop and run code for Linux on Windows. WSL1 was a Windows component that implemented Linux kernel interfaces, and didn't work great with Thunderbots code. WSL2 runs a full-fledged Linux kernel in a VM, and works great with Thunderbots and grSim with the exception that we need to use software rendering instead of GPU-accelerated rendering for grSim.

## WSL2 Setup

1. You'll need to be on an Insider build 19041.207 to use WSL2. Navigate to Settings > Windows Update > Windows Insider Program and complete the steps to enrol in the Windows Insider Program if you have not already. 
    - Being on the 'Slow' or 'Release Preview' ring will get you a new enough build for WSL2. 
2. When the Insider build finishes installing through Windows Update, you're ready to enable WSL2.
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
    - Open a PowerShell window and run command `wsl --set-default-version 2` to use WSL2 by default.
    - Install Ubuntu 18.04 LTS from the Microsoft Store.
    - Open the Ubuntu app in the Start menu. It will open a command prompt and ask you to create a new UNIX username and password for your WSL2 Ubuntu installation. 

## X Server Setup

1. [Download and install VcXsrv](https://sourceforge.net/projects/vcxsrv/files/latest/download)
2. Start WSL and use your terminal text editor of choice to edit `/etc/profile.d/wsl-integration.sh` and change the line `export LIBGL_ALWAYS_INDIRECT=1` to `# export LIBGL_ALWAYS_INDIRECT=1`
    - This comments out the line that sets the `LIBGL_ALWAYS_INDIRECT` environment variable which sets OpenGL programs to render on the X server instead of directly using the local OpenGL drivers. Unfortunately, this only supports OpenGL 1.4 which makes thunderbots give a black window and completely breaks grSim. 
    - This makes WSL use the `llvmpipe` software rasterizer instead, which will run extremely poorly on low spec machines. 
3. Start an X server by opening Xlaunch through the Start menu.
    - Choose 'Multiple windows' on the first screen
    - Choose 'Start no client' on the second screen
    - On the third screen, name sure 'Native opengl' is UNCHECKED and 'Disable access control' is CHECKED
4. Restart WSL with `wsl --shutdown` on the Windows command line. Open WSL again through the Start menu.
5. Verify that your system is configured correctly by running `glxgears -info` on the Linux command line. You should see a window pop up with spinning gears and the line `GL_RENDERER   = llvmpipe (LLVM 9.0, 256 bits)` at the top of the output.

Once you have completed all of the above, complete the [Software Setup](./getting-started.md).
