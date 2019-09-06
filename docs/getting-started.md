# Software Setup

## Table Of Contents
<!-- 
    NOTE: when creating or re-creating a table of contents like this, you can
    save a LOT of time by using this tool: 
    https://github.com/ekalinin/github-markdown-toc
-->
* [Table Of Contents](#table-of-contents)
* [Introduction](#introduction)
* [Installation and Setup](#installation-and-setup)
* [Operating Systems](#operating-systems)
* [Getting the Code](#getting-the-code)
* [Running the setup scripts](#running-the-setup-scripts)
   * [Installing Software Dependencies](#installing-software-dependencies)
   * [Installing Firmware Dependencies](#installing-firmware-dependencies)
   * [Setting Up USB Permissions](#setting-up-usb-permissions)
   * [Installing grSim](#installing-grsim)
   * [Installing CLion](#installing-clion)
      * [Getting your Student License](#getting-your-student-license)
      * [Installing CLion](#installing-clion-1)
* [Building the Code](#building-the-code)
   * [With CLion](#with-clion)
   * [From the command-line](#from-the-command-line)
* [Running the Code](#running-the-code)
   * [With CLion](#with-clion-1)
   * [From the Command Line](#from-the-command-line-1)
      * [Running Nodes Individually](#running-nodes-individually)
      * [Running Nodes Together (Running a group of Nodes)](#running-nodes-together-running-a-group-of-nodes)
* [Debugging](#debugging)

## Introduction
These instructions assume that you have the following accounts setup:
- Github
- Slack

These instructions assume you have a basic understanding of Linux and the command-line. There are many great tutorials online, such as [LinuxCommand](http://linuxcommand.org/). The most important things you'll need to know are how to move around the filesystem, and how to run programs or scripts.

## Installation and Setup

## Operating Systems

We currently only support Linux, specifically Ubuntu 18.04 LTS. You are welcome to use a different version or distribution of Linux, but may need to make some tweaks in order for things to work.

## Getting the Code

1. Open a new terminal
2. Install git by running `sudo apt-get install git`
3. Go to the [software repository](https://github.com/UBC-Thunderbots/Software)
4. Click the `Fork` button in the top-right to fork the repository
   1. Click on your user when prompted
   2. You should be automatically redirected to your new fork
5. Clone your fork of the repository (you can put it wherever you want)
   1.  Eg. `git clone https://github.com/<your-username>/Software.git`
      1. You can find this link under the green `Clone or Download` button on the main page of the Software repository
   2. We recommend cloning with SSH if you don't like typing your username and password all the time. Instructions can be found [here](https://help.github.com/articles/connecting-to-github-with-ssh/).
6. Set up your git remotes ([what is a remote and how does it work?](https://git-scm.com/book/en/v2/Git-Basics-Working-with-Remotes))
   1. You should have a remote named `origin` that points to your fork of the repository. Git will have set this up automatically when you cloned your fork in the previous step.
   2. You will need to add a second remote, named `upstream`, that points to our main Software repository, which is where you created your fork from. (**Note:** This is _not_ your fork)
      1. Open a terminal and navigate to the folder you cloned (your fork): `cd path/to/the/repository/Software`
      2. Navigate to our main Software repository in your browser and copy the url from the "Clone or Download" button. Copy the HTTPS url if you originally cloned with HTTPS, and use the SSH url if you previously cloned with SSH
      3. From your terminal, add the new remote by running `git remote add upstream <the url>` (without the angle brackets)
         1. Eg. `git remote add upstream https://github.com/UBC-Thunderbots/Software.git`
      4. That's it. If you want to double check your remotes are set up correctly, run `git remote -v` from your terminal (at the base of the repository folder again). You should see two entries: `origin` with the url for your fork of the repository, and `upstream` with the url for the main repository

## Running the setup scripts

We have several setup scripts to help you easily install the necessary dependencies in order to build and run our code. You will want to run the following scripts, which can all be found in `Software/environment_setup`

### Installing Software Dependencies

* Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
* Run `./setup_software.sh`
  * You will be prompted for your admin password
  * This script will install everything necessary in order to build and run our main `AI` software 

### Installing Firmware Dependencies

* Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
* Run `./setup_firmware.sh`
  * You will be prompted for your admin password
  * This script will install everything necessary in order to build and run our robot firmware

### Setting Up USB Permissions

* inside a terminal, navigate to the `environment_setup` folder. Eg. `cd path/to/the/repository/Software/environment_setup` 
  * Run `./setup_udev_rules.sh`
    * You will be prompted for your admin password
    * This script will set up the USB permissions required in order to use our radio/wifi dongle

### Installing grSim

* Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
* Run `./setup_grsim.sh`
  * You will be prompted for your admin password
  * This script will install `grSim`, which is the main simulator we use for development

### Installing CLion

CLion is our main IDE for editing our C/C++ code. It is designed to work with our build system, `cmake`, and has all the great features of an IDE such as code completion, syntax highlighting etc. We **strongly** recommend installing CLion and using it for development.

#### Getting your Student License

CLion is free for students, and you can use your UBC alumni email address to create a student account. If you already have a student account with JetBrains, you can skip this step.

1. If you haven't done so already, setup your UBC alumni email account [here](https://id.ubc.ca/).
2. Using your UBC email account, get a JetBrains education account [here](https://www.jetbrains.com/shop/eform/students).
   1. _JetBrains will send an initial email to confirm the UBC email you inputted. Once you have confirmed, another email will be sent to activate your new education account. You will use this account to set up CLion later on._

#### Installing CLion

* Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
* Run `./install_clion.sh`
* **Or** you can manually download and install CLion from the [JetBrains website](https://www.jetbrains.com/clion/download/)
* When you run CLion for the first time you will be prompted to enter your JetBrains account or License credentials. Use your student account.

## Building the Code

### With CLion

Our code can be built from within the CLion project just like any other project. 

1. Select the target you want to build from the dropdown list in the top right
   1. For example you could choose `ai_logic` or `backend_input`
2. Press the build button just to the left of the target dropdown list.

**NOTE:** CLion and the command line build in two seperate locations. So if you build something from the command line it will still need to build in CLion, and vice-versa

*Building in CLion is a quick and easy way to make sure parts of your code compile, but you will still need to build using `catkin_make` from the command-line if you want to run the full system. CLion builds code into `cmake-build-debug` or `cmake-build-release` folders relative to where the project was opened. ROS commands such as `roslaunch` do not look in these `cmake-build` folders for the executables to run, so if you built new code using CLion and then tried running it with `roslaunch`, your new code would not be running!*

### From the command-line

Our code can be built from the command-line by running `catkin_make` from the base of the repository. catkin_make is basically just cmake with a few extra macros. The ROS wiki has a more detailed usage guide [here](http://wiki.ros.org/catkin/commands/catkin_make).  
  
For the explanations below we assume the Software repository was cloned into the user's home directory (`~/Software`).

1. Open a terminal
2. Navigate to the base of the folder where you cloned your Software repository: cd `~/Software`
3. Simply run catkin_make: `catkin_make`

*Remember, you must build using `catkin_make` from the command line if you want to actually run the full system! Trying to run `rosrun` or `roslaunch` after building in CLion will **NOT** run the new code*

*Running `catkin_make` is what generates the `build` and `devel` folders in the Software folder. The `devel` folder is required to source the setup file (`source devel/setup.sh`), so remember to run catkin_make if those folders don't exist.*

## Running the Code

### With CLion

CLion is only able to run 1 node at a time. This node can either be standalone, or work as part of a larger group of nodes.

1. Make sure you have built the code for the node in CLion. See [the above instructions](software-setup.md#with-clion). You can either `build_all` or build the target for only the node you care about (for example, `backend_input`)
2. Make sure the selected target is the name of the node you want to run, eg. `backend_input`
3. Press the button in the top right that looks like a "Play" button to run the code (it should be right next to where you selected the target to build/run)
4. Any console output will be displayed in CLion's window

If you want to run the node alongside other nodes, run them using `rosrun` from the command line. [See below for running noes via command line](software-setup.md#running-nodes-individually).

### From the Command Line

#### Running Nodes Individually

ROS nodes can be run individually using [rosrun](http://wiki.ros.org/rosbash#rosrun). The syntax is typically `rosrun PACKAGE_NAME NODE_NAME`. 

1. Open a new terminal. Navigate to the `Software` folder and source `devel/setup.sh`
   1. We will call this _terminal A_
2. Run `roscore`
   1. This runs the ROS Master, which is the service that contains all the IP lookup tables and other networking information that allows the ROS nodes to "find" each other and communicate
3. Open a new terminal. Navigate to the `Software` folder and source `devel/setup.sh`
   1. We will call this _terminal B_
4. Lets say you want to run the `network_input` node. We know this node is in the `thunderbots` package, so using the syntax mentioned above we can run: `rosrun thunderbots network_input`
5. _terminal B_ will now be running the network_input node. You will see any print statements the node has.

#### Running Nodes Together (Running a group of Nodes)

[roslaunch](http://wiki.ros.org/roslaunch/Commandline%20Tools#roslaunch) is the ROS command to run a group of nodes. This group of nodes is defined in a launch file, which is file that uses XML syntax to list the nodes (and their parameters) that should be run, and has a   
`.launch` extension. For example, we have `ai_grsim.launch`.  
  
The typical syntax for `roslaunch` is `roslaunch <packagename> <name of launch file>`. To run a launch file:

1. Open a new terminal. Navigate to the `Software` folder and source `devel/setup.sh`
2. Lets assume we want to run our main AI for grSim. This means we will want to run the `ai_grsim.launch` launch file, which we know is in the `thunderbots` package. Then using the syntax above, we can run: `roslaunch thunderbots ai.launch`
3. Your terminal will now be running all the nodes defined in the ai.launch file, and will be displaying any print statements from all of the nodes

*Note that we don't need to run `roscore` in this case. `roslaunch` takes care of that for us*

*Note that there is no way to run a launch file or a group of nodes from CLion (without perhaps using custom targets and arguments). Using `roslaunch` from the command-line is the recommended way of running multiple nodes, and is the way our programs should be run if you are trying do some field tests or run the full system to play a game.*

## Debugging

The easiest way to debug the code is running it with CLion. The steps are exactly the same as [running a node with CLion](software-setup.md#with-clion-1), except you press the Debug button rather than the Run button to run the code in debug mode. You will then be able to set breakpoints, step through the code, and use all the functionality of the CLion debugger.

*Remember that you can only debug one node at a time. Be sure to run the node that contains the code you are trying to debug.*
