# Software Setup

## Table Of Contents
<!-- 
    NOTE: when creating or re-creating a table of contents like this, you can
    save a LOT of time by using this tool: 
    https://github.com/ekalinin/github-markdown-toc
-->
- [Software Setup](#software-setup)
  - [Table Of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Installation and Setup](#installation-and-setup)
  - [Operating Systems](#operating-systems)
  - [Getting the Code](#getting-the-code)
  - [Running the setup scripts](#running-the-setup-scripts)
    - [Installing Software Dependencies](#installing-software-dependencies)
    - [Installing an IDE](#installing-an-ide)
      - [CLion](#clion)
        - [Getting your Student License](#getting-your-student-license)
        - [Installing CLion](#installing-clion)
      - [VSCode](#vscode)
  - [Building and Running the Code](#building-and-running-the-code)
    - [Building from the command-line](#building-from-the-command-line)
         - [Using the fuzzy finder](#using-the-fuzzy-finder)
         - [Editing with Vim or NeoVim](#editing-with-vim-or-neovim)
    - [Building with CLion](#building-with-clion)
    - [With VSCode](#with-vscode)
    - [Running our AI, Simulator, SimulatedTests or Robot Diagnostics](#running-our-ai-simulator-simulatedtests-or-robot-diagnostics)
    - [Running AI vs AI](#running-ai-vs-ai)
  - [Debugging](#debugging)
    - [Debugging with CLion](#debugging-with-clion)
    - [Debugging from the Command line](#debugging-from-the-command-line)
  - [Profiling](#profiling)
  - [Building for Jetson Nano](#building-for-jetson-nano)
  - [Deploying Robot Software to the Jetson Nano ](#deploying-to-jetson-nano)
  - [Setting up Virtual Robocup 2021](#setting-up-virtual-robocup-2021)
    - [Setting up the SSL Simulation Environment](#setting-up-the-ssl-simulation-environment)
    - [Pushing a Dockerfile to dockerhub](#pushing-a-dockerfile-to-dockerhub)

## Introduction
These instructions assume that you have the following accounts setup:
- [Github](https://github.com/login)
- [Slack](https://thunderbots.slack.com/)

These instructions assume you have a basic understanding of Linux and the command-line. There are many great tutorials online, such as [LinuxCommand](http://linuxcommand.org/). The most important things you'll need to know are how to move around the filesystem, and how to run programs or scripts.

## Installation and Setup

## Operating Systems

We currently only support Linux, specifically Ubuntu 18.04 LTS and 20.04 LTS (CI tests on 20.04). You are welcome to use a different version or distribution of Linux, but may need to make some tweaks in order for things to work.

You can use Ubuntu 20.04 LTS inside Windows through Windows Subsystem for Linux, by following [this guide](./getting-started-wsl.md). **Running and developing Thunderbots on Windows is experimental and not officially supported.**

## Getting the Code

1. Open a new terminal
2. Install git by running `sudo apt-get install git`
3. Go to the [software repository](https://github.com/UBC-Thunderbots/Software)
4. Click the `Fork` button in the top-right to fork the repository ([click here to learn about Forks](https://help.github.com/en/articles/fork-a-repo))
   1. Click on your user when prompted
   2. You should be automatically redirected to your new fork
5. Clone your fork of the repository. As GitHub is forcing users to stop using usernames and passwords, we will be using the SSH link.  Returning members who are migrating to using SSH after cloning from a previous method can use the following instructions to set up a new local repository using SSH.
   1. To connect to GitHub using SSH, if not setup prior, you will need to add an SSH key to your GitHub account. Instructions can be found [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).  For each computer you contribute to GitHub with, you will need an additional SSH Key pair linked to your account.
   2.  After you have successfully set up an SSH Key for your device and added it to GitHub, you can clone the repository using the following command (you can put it wherever you like):
        1.  Eg. `git clone git@github.com:<your_username>/Software.git`
        2.  You can find this link under the green `Clone or Download` button on the main page of the Software repository, under the SSH tab.  (This should now be available after adding your SSH key to GitHub successfully.)
6. Set up your git remotes ([what is a remote and how does it work?](https://git-scm.com/book/en/v2/Git-Basics-Working-with-Remotes))
   1. You should have a remote named `origin` that points to your fork of the repository. Git will have set this up automatically when you cloned your fork in the previous step.
   2. You will need to add a second remote, named `upstream`, that points to our main Software repository, which is where you created your fork from. (**Note:** This is _not_ your fork)
      1. Open a terminal and navigate to the folder you cloned (your fork): `cd path/to/the/repository/Software`
      2. Navigate to our main Software repository in your browser and copy the url from the "Clone or Download" button. Copy the HTTPS url if you originally cloned with HTTPS, and use the SSH url if you previously cloned with SSH
      3. From your terminal, add the new remote by running `git remote add upstream <the url>` (without the angle brackets)
         1. Eg. `git remote add upstream https://github.com/UBC-Thunderbots/Software.git`
      4. That's it. If you want to double check your remotes are set up correctly, run `git remote -v` from your terminal (at the base of the repository folder again). You should see two entries: `origin` with the url for your fork of the repository, and `upstream` with the url for the main repository

*See our [workflow document](workflow.md) for how to use git to make branches, submit Pull Requests, and track issues*

## Running the setup scripts

We have several setup scripts to help you easily install the necessary dependencies in order to build and run our code. You will want to run the following scripts, which can all be found in `Software/environment_setup`

### Installing Software Dependencies

* Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
* Run `./setup_software.sh`
  * You will be prompted for your admin password
  * This script will install everything necessary in order to build and run our main `AI` software 

### Installing an IDE

For those who prefer working on C/C++ with an IDE, we provide two options: [CLion](#clion) for an integrated experience and [VSCode](vscode) for a more lightweight setup. Both support our build system `bazel`.

#### CLion

CLion is the most full-featured IDE, with code completion, code navigation, and integrated building, testing, and debugging.

##### Getting your Student License

CLion is free for students, and you can use your UBC alumni email address to create a student account. If you already have a student account with JetBrains, you can skip this step.

1. If you haven't done so already, setup your UBC alumni email account [here](https://id.ubc.ca/).
2. Using your UBC email account, get a JetBrains education account [here](https://www.jetbrains.com/shop/eform/students).
   1. _JetBrains will send an initial email to confirm the UBC email you inputted. Once you have confirmed, another email will be sent to activate your new education account. You will use this account to set up CLion later on._

##### Installing CLion

* Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
* Run `./install_clion.sh` (* **DO NOT** download CLion yourself unless you know what you're doing. The `install_clion.sh` script will grab the correct version of CLion and the Bazel plugin to ensure everything is compatible *).
* When you run CLion for the first time you will be prompted to enter your JetBrains account or License credentials. Use your student account.

#### VSCode

VSCode is the more lightweight IDE, with support for code navigation, code completion, and integrated building and testing. However, debugging isn't integrated into this IDE.

1. Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
2. Run `./install_vscode.sh` (* **DO NOT** download VSCode yourself unless you know what you're doing. The `install_vscode.sh` script will grab the most stable version of VSCode *)
3. Open `vscode`. You can type `vscode` in the terminal, or click the icon on your Desktop.
&. Click `Open Folder` and navigate to where you cloned software. So if I cloned the repo to `/home/my_username/Downloads/Software`, I would select `/home/my_username/Downloads/Software`.
4. VSCode will prompt you to install recommended extensions, click `Install`, this installs necessary plugins to work on the codebase. (Bazel, C++, Python, etc..)
5. Navigate to File -> Preferences -> Settings -> Workspace -> Extensions -> Bazel and select the `Bazel: Enable Code Lens` option.

## Building and Running the Code

### Building from the command-line

1. Navigate to the root of this repository (wherever you have it cloned on your computer)
2. Navigate to `src`.
3. Build a specific target for running (for example): `bazel build //software/geom:angle_test`
4. Run a specific target by running (for example): `bazel run //software/geom:angle_test`
5. Run a specific *test* by running (for example): `bazel test //software/geom:angle_test`
6. Build everything by running `bazel build //...`
7. Run all the tests by running `bazel test //...`
*See the bazel [command-line docs](https://docs.bazel.build/versions/master/command-line-reference.html) for more info.*
*Note: the targets are defined in the BUILD files in our repo*

#### Using the fuzzy finder
We have a ./tbots.py script in the src folder that will fuzzy find for targets. For example, 

1. Build a specific target for running (for example): `./tbots.py build angletest`
2. Run a specific target by running (for example): `./tbots.py run goalietest -t`
3. Run a specific *test* by running (for example): `./tbots.py test goalietest -t`

where the `-t` flag indicates whether Thunderscope should be launched. Run `./tbots.py --help` for more info

#### Editing with Vim or NeoVim

When editing with Vim or NeoVim, it's helpful to use plugins, such as [COC](https://github.com/neoclide/coc.nvim) or [LSP](https://github.com/neovim/nvim-lspconfig) to find references, go-to-definition, autocompletion, and more.
These tools require a `compile_commands.json` file, which can be generated by following these instructions:
1. Symlink `src/external` to `bazel-out/../../../external`: `ln -s bazel-out/../../../external .` from the src folder
2. Generate the `compile_commands.json` file: `bazel run //:refresh_compile_commands`.

### Building with CLion

First we need to setup CLion
1. Open CLion
2. Select `Import Bazel Project`
3. Set `Workspace` to wherever you cloned the repository + `/src`. So if I cloned the repo to `/home/my_username/Downloads/Software`, my workspace would be `/home/my_username/Downloads/Software/src`.
4. Select `Import project view file`, and select the file `.bazelproject` (which will be under the `src` folder)
5. Click `Next`
6. Change the Project Name to whatever you want. Leave everything else as it is ("Use shared project view file" should be selected).
7. Click `Finish` and you're good to go! Give CLion some time to find everything in your repo.

Now that you're setup, if you can run it on the command line, you can run it in clion. There are two main ways of doing so.
1. Open any `BUILD` file and right clight in a `cc_library()` call. This will give you the option to `Run` or `Debug` that specific target. Try it by opening `Software/src/software/geom/BUILD` and right-clicking on the `cc_library` for `angle_test`!
2. Add a custom build configuration (more powerful, so make sure you understand this!)
    1. Select `Add Configuration` from the drop-down in the top-right of CLion
    2. Click on `+`, choose `Bazel Command`.
    3. For `Target Expression`, you can put anything that comes after a `build`, `run`, `test`, etc. call on the command line. For example: `//software/geom:angle_test`.
    4. For `Bazel Command` you can put any bazel command, like `build`, `run`, `test`, etc.
    5. Click `Ok`, then there should be a green arrow in the top right corner by the drop-down menu. Click it and the test will run!
 
### With VSCode
1. Open VSCode
2. Navigate to `Software/src/software/geom/BUILD`
3. On top of every `cc_test`, `cc_library` and `cc_binary` there should be a `Test ...`, `Build ...` or `Run ...` for the respective target.
4. Click `Test //software/geom:angle_test` to run the `angle_test`

### Running our AI, Simulator, SimulatedTests or Robot Diagnostics

1. Open your terminal, `cd` into `Software/src` and run `ifconfig`.
2. Pick the network interface you would like to use:
    1. If you are running things locally, you can pick any interface that is not `lo`
    2. If you would like to communicate with robots on the network, make sure to select the interface that is connected to the same network as the robots.
3. Run our AI: `bazel run //software:full_system -- --interface=[interface_here] --backend=SimulatorBackend`
    - This will launch the Visualizer, which displays what the AI is currently "seeing" and allows us to interact with the AI through the dynamic parameters.
    - The field should be empty, as we are currently not receiving SSL Vision packets.
4. Run our Simulator: `bazel run //software:standalone_simulator_main -- --interface=[interface_here]`
    - The Simulator runs our firmware and Box2D (a physics engine) to simulate how our robots would behave on the field.
    - The Simulator outputs SSL Vision packets, which contain position information of all robots and the ball.
    - Our AI can now "see" the robots, and they should be displayed on the Visualizer.
    - You can use ctrl-click to move the ball around in the Simulator, and try changing the Play Override on the Visualizer to see the robots move!
5. Run Robot Diagnostics: `bazel run //software:robot_diagnostics:robot_diagnostics_main -- --interface=[interface_here] --backend=WifiBackend`
    - The Mechanical and Electrical sub-teams use Robot Diagnostics to test specific parts of the Robot.
6. Run our SimulatedPlayTests in the visualizer: `bazel test //software/ai/hl/stp/play:[some_target_here] --test_arg="--enable_visualizer"` or `bazel run //software/ai/hl/stp/play:[some_target_here] -- --enable_visualizer`
    - This will launch the visualizer and simulate AI Plays, allowing us to visually see the robots acting according to their roles.
7. Run our SimulatedTacticTests in the visualizer: `bazel test //software/ai/hl/stp/tactic:[some_target_here] --test_arg="--enable_visualizer"` or `bazel run //software/ai/hl/stp/tactic:[some_target_here] -- --enable_visualizer`
    - This will launch the visualizer and simulate AI Tactic on a single robot

** NOTE: If you want to run SimulatedTests with the AI initially stopped, then use the `--stop_ai_on_start` flag ** 

### Running AI vs AI
1. Open your terminal, `cd` into `Software/src`
2. Run `./tbots.py run thunderscope`

## Debugging
Debugging from the command line is certainly possible, but debugging in a full IDE is *really* nice (plz trust us). 

### Debugging with CLion
Debugging in CLion is as simple as running the above instructions for building CLion, but clicking the little green bug in the top right corner instead of the little green arrow!

### Debugging from the Command line
To debug from the command line, first you need to build your target with the debugging flag - `bazel build -c dbg //some/target:here`. When the target builds, you should see a path `bazel-bin/<target>`. Copy that path, and run `gdb <path>`. Please see [here](https://www.cs.cmu.edu/~gilpin/tutorial/) for a tutorial on how to use `gdb` if you're not familiar with it. Alternatively, you could do `bazel run -c dbg --run_under="gdb" //some/target:here`, which will run the target in `gdb`. While this is taken directly from the bazel docs, gdb may sometimes hang when using `--run_under`, so building the target first with debugging flags and running afterwards is preferred.


## Profiling 
Unfortunately profiling for Bazel targets is not supported in CLion at this time. Hence the only way is via command line. Use the following command:
```
bazel run -c dbg --run_under="valgrind --tool=callgrind --callgrind-out-file=/ABSOLUTE/PATH/TO/profile.callgrind" //target/to:run

// Example
bazel run -c dbg --run_under="valgrind --tool=callgrind --callgrind-out-file=/tmp/profile.callgrind" //software/geom:angle_test
```
This will output the file at the _absolute_ path given via the `--callgrind-out-file` argument. This file can then be viewed using `kcachegrind` (example: `kcachegrind /tmp/profile.callgrind`), giving lots of useful information about where time is being spent in the code.

## Building for Jetson Nano 

To build for the Jetson Nano, build the target with the `--cpu=jetson_nano` flag and the toolchain will automatically build using the ARM toolchain for Jetson Nano. For example, `bazel build --cpu=jetson_nano //software/geom/...`.


## Deploying to Jetson Nano 

We use ansible to automatically update software running on the Jetson Nano. [See these instructions.](../src/software/jetson_nano/ansible/README.md) 

## Setting up Virtual Robocup 2021

### Setting up the SSL Simulation Environment

1. Fork the [SSL-Simulation-Setup](https://github.com/RoboCup-SSL/ssl-simulation-setup) repository.  
2. Clone it.
3. Follow these [instructions](https://github.com/RoboCup-SSL/ssl-simulation-setup/blob/master/Readme.md) to set up and run the repository.

### Pushing a Dockerfile to dockerhub

After editing the dockerfile, build the image and push it to dockerhub with the following steps

1. To build the image, make sure that you are in the same directory as your image, and then run `docker build -t ubcthunderbots/<image name>[:tag] .` Make sure that your chosen image name matches a repository in dockerhub. Here's an example with the robocup 2021 setup image: `docker build -t ubcthunderbots/tbots-software-env:0.0.1`
2. Now, push your image to dockerhub. Get the credentials for the thunderbots dockerhub account from a software lead.
   1. Log into the docker account with `docker login`. You will be prompted for a username and password
   2. Now, push this image by its name: `docker push ubcthunderbots/<image name>[:tag]`

