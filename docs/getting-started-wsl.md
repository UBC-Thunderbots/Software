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

1. [Download VcXsrv](https://sourceforge.net/projects/vcxsrv/files/latest/download)
