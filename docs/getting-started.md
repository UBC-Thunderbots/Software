Table of Contents                                                                                                                                                                            
=================                                                                                                                                                                            
 <!-- 
    NOTE: when creating or re-creating a table of contents like this, you can
    save a LOT of time by using this tool: 
    https://github.com/ekalinin/github-markdown-toc
-->

* [Software Setup](#software-setup)                                                                                                                                                          
   * [Introduction](#introduction)                                                                                                                                                           
   * [Installation and Setup](#installation-and-setup)                                                                                                                                       
      * [Operating systems](#operating-systems)                                                                                                                                              
      * [Getting the Code](#getting-the-code)                                                                                                                                                
      * [Installing Software Dependencies](#installing-software-dependencies)
      * [Installing an IDE](#installing-an-ide)
         * [Installing an IDE: CLion](#installing-an-ide-clion)
            * [Getting your Student License](#getting-your-student-license)
            * [Installing CLion](#installing-clion)
      * [Installing an IDE: VSCode](#installing-an-ide-vscode)
      * [Editing with Vim or NeoVim](#editing-with-vim-or-neovim)
   * [Building and Running the Code](#building-and-running-the-code)
      * [Building from the command-line](#building-from-the-command-line)
      * [Building from the command-line using the fuzzy finder](#building-from-the-command-line-using-the-fuzzy-finder)
      * [Building with CLion](#building-with-clion)
      * [Building with VSCode](#building-with-vscode)
      * [Running our AI, Simulator, SimulatedTests or Robot Diagnostics](#running-our-ai-simulator-simulatedtests-or-robot-diagnostics)
   * [Debugging](#debugging)
      * [Debugging with CLion](#debugging-with-clion)
      * [Debugging from the Command line](#debugging-from-the-command-line)
   * [Profiling](#profiling)
   * [Building for Jetson Nano](#building-for-jetson-nano)
   * [Deploying to Jetson Nano](#deploying-to-jetson-nano)
   * [Setting up Virtual Robocup 2021](#setting-up-virtual-robocup-2021)
      * [Setting up the SSL Simulation Environment](#setting-up-the-ssl-simulation-environment)
      * [Pushing a Dockerfile to dockerhub](#pushing-a-dockerfile-to-dockerhub)
* [Workflow](#workflow)
   * [Issue and Project Tracking](#issue-and-project-tracking)
      * [Issues](#issues)
   * [Git Workflow](#git-workflow)
      * [Forking and Branching](#forking-and-branching)
      * [Creating a new Branch](#creating-a-new-branch)
      * [Making Commits](#making-commits)
      * [Updating Your Branch and Resolving Conflicts](#updating-your-branch-and-resolving-conflicts)
      * [Formatting Your Code](#formatting-your-code)
      * [Pull Requests](#pull-requests)
      * [Reviewing Pull Requests](#reviewing-pull-requests)
   * [Example Workflow](#example-workflow)
   * [Testing](#testing)

# Software Setup

## Introduction

These instructions assume that you have the following accounts setup:
- [Github](https://github.com/login)
- [Discord](https://discord.com). Please contact a Thunderbots lead to receive the invite link.

These instructions assume you have a basic understanding of Linux and the command-line. There are many great tutorials online, such as [LinuxCommand](http://linuxcommand.org/). The most important things you'll need to know are how to move around the filesystem, and how to run programs or scripts.

## Installation and Setup

### Operating systems

We currently only support Linux, specifically Ubuntu 20.04 LTS. You are welcome to use a different version or distribution of Linux, but may need to make some tweaks in order for things to work.

You can use Ubuntu 20.04 LTS inside Windows through Windows Subsystem for Linux, by following [this guide](./getting-started-wsl.md). **Running and developing Thunderbots on Windows is experimental and not officially supported.**

### Getting the Code

1. Open a new terminal
2. Install git by running `sudo apt-get install git`
3. Go to the [software repository](https://github.com/UBC-Thunderbots/Software)
4. Click the `Fork` button in the top-right to fork the repository ([click here to learn about Forks](https://help.github.com/en/articles/fork-a-repo))
   1. Click on your user when prompted
   2. You should be automatically redirected to your new fork
5. Clone your fork of the repository. As GitHub is forcing users to stop using usernames and passwords, we will be using the SSH link.  Returning members who are migrating to using SSH after cloning from a previous method can use the following instructions to set up a new local repository using SSH.
   1. To connect to GitHub using SSH, if not setup prior, you will need to add an SSH key to your GitHub account. Instructions can be found [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).  For each computer you contribute to GitHub with, you will need an additional SSH Key pair linked to your account.
   2.  After you have successfully set up a SSH key for your device and added it to GitHub, you can clone the repository using the following command (you can put it wherever you like):
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

*See our [workflow](#workflow) for how to use git to make branches, submit Pull Requests, and track issues*

### Installing Software Dependencies

We have several setup scripts to help you easily install the necessary dependencies in order to build and run our code. You will want to run the following scripts, which can all be found in `Software/environment_setup`

* Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
* Run `./setup_software.sh`
  * You will be prompted for your admin password
  * This script will install everything necessary in order to build and run our main `AI` software 

### Installing an IDE

For those who prefer working on C/C++ with an IDE, we provide two options: CLion for an integrated experience and VSCode for a more lightweight setup. Both support our build system `bazel`.

#### Installing an IDE: CLion

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

### Installing an IDE: VSCode

VSCode is the more lightweight IDE, with support for code navigation, code completion, and integrated building and testing. However, debugging isn't integrated into this IDE.

1. Inside a terminal, navigate to the environment_setup folder. Eg. `cd path/to/the/repository/Software/environment_setup`
2. Run `./install_vscode.sh` (* **DO NOT** download VSCode yourself unless you know what you're doing. The `install_vscode.sh` script will grab the most stable version of VSCode *)
3. Open `vscode`. You can type `vscode` in the terminal, or click the icon on your Desktop.
&. Click `Open Folder` and navigate to where you cloned software. So if I cloned the repo to `/home/my_username/Downloads/Software`, I would select `/home/my_username/Downloads/Software`.
4. VSCode will prompt you to install recommended extensions, click `Install`, this installs necessary plugins to work on the codebase. (Bazel, C++, Python, etc..)
5. Navigate to File -> Preferences -> Settings -> Workspace -> Extensions -> Bazel and select the `Bazel: Enable Code Lens` option.

### Editing with Vim or NeoVim

When editing with Vim or NeoVim, it's helpful to use plugins, such as [COC](https://github.com/neoclide/coc.nvim) or [LSP](https://github.com/neovim/nvim-lspconfig) to find references, go-to-definition, autocompletion, and more.
These tools require a `compile_commands.json` file, which can be generated by following these instructions:
1. Symlink `src/external` to `bazel-out/../../../external`: `ln -s bazel-out/../../../external .` from the src folder
2. Generate the `compile_commands.json` file: `bazel run //:refresh_compile_commands`.


## Building and Running the Code

### Building from the command-line

1. Navigate to the root of this repository (wherever you have it cloned on your computer)
2. Navigate to `src`.
3. Build a specific target for running (for example): `bazel build //software/geom:angle_test`
4. Run a specific target by running (for example): `bazel run //software/geom:angle_test`
5. Run a specific *test* by running (for example): `bazel test //software/geom:angle_test`
6. Build everything by running `bazel build //...`
7. Run all the tests by running `bazel test //...`

*See the Bazel [command-line docs](https://bazel.build/reference/command-line-reference) for more info.*
*Note: the targets are defined in the BUILD files in our repo*

### Building from the command-line using the fuzzy finder

We have a ./tbots.py test runner script in the src folder that will fuzzy find for targets. For example, 

1. Build a specific target for running (for example): `./tbots.py build angletest`
2. Run a specific target by running (for example): `./tbots.py run goalietactictest -t`
3. Run a specific *test* by running (for example): `./tbots.py test goalietactictest -t`

where the `-t` flag indicates whether Thunderscope should be launched. Run `./tbots.py --help` for more info

### Building with CLion

First we need to setup CLion
1. Open CLion
2. Select `Import Bazel Project`
3. Set `Workspace` to wherever you cloned the repository + `/src`. So if I cloned the repo to `/home/my_username/Downloads/Software`, my workspace would be `/home/my_username/Downloads/Software/src`.
4. Select `Import project view file`, and select the file `.bazelproject` (which will be under the `src` folder)
5. Click `Next`
6. Change the Project Name to whatever you want. Leave everything else as it is ("Use shared project view file" should be selected).
7. Click `Finish` and you're good to go! Give CLion some time to find everything in your repo.

Now that you're setup, if you can run it on the command line, you can run it in CLion. There are two main ways of doing so.
1. Open any `BUILD` file and right clight in a `cc_library()` call. This will give you the option to `Run` or `Debug` that specific target. Try it by opening `Software/src/software/geom/BUILD` and right-clicking on the `cc_library` for `angle_test`!
2. Add a custom build configuration (more powerful, so make sure you understand this!)
    1. Select `Add Configuration` from the drop-down in the top-right of CLion
    2. Click on `+`, choose `Bazel Command`.
    3. For `Target Expression`, you can put anything that comes after a `build`, `run`, `test`, etc. call on the command line. For example: `//software/geom:angle_test`.
    4. For `Bazel Command` you can put any Bazel command, like `build`, `run`, `test`, etc.
    5. Click `Ok`, then there should be a green arrow in the top right corner by the drop-down menu. Click it and the test will run!

### Building with VSCode

1. Open VSCode
2. Navigate to `Software/src/software/geom/BUILD`
3. On top of every `cc_test`, `cc_library` and `cc_binary` there should be a `Test ...`, `Build ...` or `Run ...` for the respective target.
4. Click `Test //software/geom:angle_test` to run the `angle_test`

### Running our AI, Simulator, SimulatedTests or Robot Diagnostics

1. Run our AI on Thunderscope:
    - Thunderscope is the software that coordinates our AI, Simulator, Visualizer and RobotDiagnostics
    - After launching Thunderscope, we can see what the AI is currently "seeing" and interact with it through dynamic parameters. 
    - If we want to run with simulated AI vs AI:
        - `./tbots.py run thunderscope_main` will start Thunderscope with our Simulator, a blue FullSystem and yellow FullSystem. Each FullSystem contains the respective AI for each side. The command will start Thunderscope and set up communication between the Simulator, GameController and FullSystems.
        - We use ER Force's Simulator to simulate how our robots would behave on the field. This simulator is powerful because it includes vision noise, allowing us to further stress test our gameplay.
        - The Simulator outputs SSL Vision packets, which contain position information of all robots and the ball.
        - Our AI can now "see" the robots, and they should be displayed on the Visualizer.
        - Currently, there should be six blue robots and six yellow robots on screen. All these robots are probably stationary because there are no RefereeCommands to respond to. We can change this state using the GameController. In Thunderscope, navigate to the "GameController" tab at the top or alternatively, type `localhost:8081` in your browser. Here, we should see the GameController page with two columns of buttons on the left: one representing commands for the Yellow team and one for the Blue team. We can control gameplay by issuing RefereeCommands.
            - To start normal gameplay from Kickoff, press "Stop", then "Kickoff" for either team and then "Normal Start".
            - To learn more about how we coordinate different RefereeCommands to start special case gameplay behaviour (PenaltyKick, CornerKick, FreeKick), look at the [SSL rule documentation](https://ssl.robocup.org/rules/).
        - In addition, you can use ctrl-click to move the ball around in the Simulator, or try changing the Play Override on the Visualizer to select specific Plays!

    - If we want to run it with real robots:
        - Open your terminal, `cd` into `Software/src` and run `ifconfig`.
            - Pick the network interface you would like to use:
                1. If you are running things locally, you can pick any interface that is not `lo`
                2. If you would like to communicate with robots on the network, make sure to select the interface that is connected to the same network as the robots.
            - For example, on a sample machine, the output may look like this:

                ```
                enp0s5: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
                        ...
                        [omitted]
                        ...

                lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
                        ...
                        [omitted]
                        ...
                ```

            - An appropriate interface we could choose is `enp0s5`
        - If we are running the AI as "blue": `./tbots.py run thunderscope_main --interface=[interface_here] --run_blue`
        - If we are running the AI as "yellow": `./tbots.py run thunderscope_main --interface=[interface_here] --run_yellow`
        - `[interface_here]` corresponds to the `ifconfig` interfaces seen in the previous step
            - For instance, a call to run the AI as blue on wifi could be: `./tbots.py run thunderscope_main --interface=enp0s5 --run_blue`
        - This command will set up robot communication and the Unix full system binary context manager. The Unix full system context manager hooks up our AI, Backend and SensorFusion
2. Run Robot Diagnostics:
    - (#2711) There isn't a clean way to do this at the moment. This command is subject to change.
    - The Mechanical and Electrical sub-teams use Robot Diagnostics to test specific parts of the Robot.
    - `./tbots.py run thunderscope --run_blue --interface <network_interface>`
    - [More info here](useful-robot-commands#robot-diagnostics)
3. Run our SimulatedPlayTests in Thunderscope
    - This will launch the visualizer and simulate AI Plays, allowing us to visually see the robots acting according to their roles.
    1. For legacy C++ tests (#2581) with the visualizer:
        1. First run Thunderscope configured for receiving protobufs over unix sockets correctly: `./tbots.py run thunderscope_main --visualize_cpp_test`
        2. Then run `./tbots.py test [some_target_here] --run_sim_in_realtime`
    2. For PyTests:
        - With the visualizer: `./tbots.py test [some_target_here] -t`
        - Without the visualizer: `./tbots.py test [some_target_here]`
    3. For legacy C++ tests (#2581) without the visualizer:
        - `./tbots.py test [some_target_here]`
4. Run our SimulatedTacticTests in Thunderscope:
    - This will launch the visualizer and simulate an AI Tactic on a single robot
    1. For legacy C++ tests (#2581) with the visualizer:
        - First, run Thunderscope configured for receiving protobufs over unix sockets correctly: `./tbots.py run thunderscope_main --visualize_cpp_test`
        - Then run `./tbots.py test [some_target_here] --run_sim_in_realtime`
    2. For PyTests:
        - With the visualizer: `./tbots.py test [some_target_here] -t`
        - Without the visualizer: `./tbots.py test [some_target_here]`
    3. For legacy C++ tests (#2581) without the visualizer:
        - `./tbots.py test [some_target_here]`

## Debugging

Debugging from the command line is certainly possible, but debugging in a full IDE is *really* nice (plz trust us). 

### Debugging with CLion

Debugging in CLion is as simple as running the above instructions for building CLion, but clicking the little green bug in the top right corner instead of the little green arrow!

### Debugging from the Command line

To debug from the command line, first you need to build your target with the debugging flag - `bazel build -c dbg //some/target:here`. When the target builds, you should see a path `bazel-bin/<target>`. Copy that path, and run `gdb <path>`. Please see [here](https://www.cs.cmu.edu/~gilpin/tutorial/) for a tutorial on how to use `gdb` if you're not familiar with it. Alternatively, you could do `bazel run -c dbg --run_under="gdb" //some/target:here`, which will run the target in `gdb`. While this is taken directly from the Bazel docs, gdb may sometimes hang when using `--run_under`, so building the target first with debugging flags and running afterwards is preferred.

## Profiling 

Profiling is an optimization tool used to identify the time and space used by code, with a detailed breakdown to help identify areas of potential performance improvements. Unfortunately profiling for Bazel targets is not supported in CLion at this time. Hence the only way is via command line. Use the following command:
```
bazel run -c dbg --run_under="valgrind --tool=callgrind --callgrind-out-file=/ABSOLUTE/PATH/TO/profile.callgrind" //target/to:run

// Example
bazel run -c dbg --run_under="valgrind --tool=callgrind --callgrind-out-file=/tmp/profile.callgrind" //software/geom:angle_test
```

This will output the file at the _absolute_ path given via the `--callgrind-out-file` argument. This file can then be viewed using `kcachegrind` (example: `kcachegrind /tmp/profile.callgrind`), giving lots of useful information about where time is being spent in the code.

## Building for Jetson Nano 

To build for the Jetson Nano, build the target with the `--cpu=jetson_nano` flag and the toolchain will automatically build using the ARM toolchain for Jetson Nano. For example, `bazel build --cpu=jetson_nano //software/geom/...`.

## Deploying to Jetson Nano 

We use ansible to automatically update software running on the Jetson Nano. [More info here.](useful-robot-commands.md#flashing-the-nano) 

To update binaries on a working robot, you can run:

`bazel run //software/jetson_nano/ansible:run_ansible --cpu=jetson_nano -- --playbook deploy_nano.yml --hosts <robot_ip> --ssh_pass <jetson_nano_password>`

## Setting up Virtual Robocup 2021

### Setting up the SSL Simulation Environment

1. Fork the [SSL-Simulation-Setup](https://github.com/RoboCup-SSL/ssl-simulation-setup) repository.  
2. Clone it.
3. Follow these [instructions](https://github.com/RoboCup-SSL/ssl-simulation-setup/blob/master/Readme.md) to set up and run the repository.

### Pushing a Dockerfile to dockerhub

After editing the dockerfile, build the image and push it to dockerhub with the following steps

1. To build the image, make sure that you are in the same directory as your image, and then run `docker build -t ubcthunderbots/<image name>[:tag] .` Make sure that your chosen image name matches a repository in dockerhub. Here's an example with the Robocup 2021 setup image: `docker build -t ubcthunderbots/tbots-software-env:0.0.1`
2. Now, push your image to dockerhub. Get the credentials for the Thunderbots dockerhub account from a software lead.
   1. Log into the docker account with `docker login`. You will be prompted for a username and password
   2. Now, push this image by its name: `docker push ubcthunderbots/<image name>[:tag]`

# Workflow

## Issue and Project Tracking

We try keep our issue and project tracking fairly simple, to reduce the overhead associated with tracking all the information and to make it easier to follow. If you are unfamiliar with GitHub issues, [this article](https://guides.github.com/features/issues/) gives a good overview.

### Issues

We use issues to keep track of bugs in our system, and new features or enhancements we want to add. When creating a new issue, we have a simple "Task" template that can be filled out. We *strongly* recommend using the template since it provides guiding questions/headings to make sure we have all the necessary information in each issue.

*It is very important to give lots of detail and context when creating an issue. It is best to pretend you are writing the issue for someone who has not worked on the relevant part of the system before, and to leave a good enough explanation that someone with very little prior knowledge could get started. Sometimes issues get worked on many months after they were created, and we don't want to forget exactly what we wanted to do and why.*

In general if you find an issue with the system, first check with others on your team to make sure that this is indeed unintended behavior (you never know), and make sure that an issue has not already been created before you create a new one.  
  
The same goes for feature requests. Just make sure that whatever you want to say doesn't already exist in an issue.

## Git Workflow

### Forking and Branching

In general, we follow the Forking Workflow

* [What it is](https://www.atlassian.com/git/tutorials/comparing-workflows#forking-workflow)
* [How to use it](https://gist.github.com/Chaser324/ce0505fbed06b947d962)
* Instructions on obtaining your own Fork of our repository can be found in the [Getting the Code](#getting-the-code) section.

### Creating a new Branch

For each Issue of project you are working on, you should have a separate branch. This helps keep work organized and separate.

**Branches should always be created from the latest code on the `master` branch of our main Software repository**. If you followed the steps in [Installation and Setup](#installation-and-setup), this will be `upstream/master`. Once this branch is created, you can push it to your fork and update it with commits until it is ready to merge. 

1. Navigate to the base folder of your Software repository: `cd path/to/the/repository/Software`
2. Make git aware of any new changes to `upstream` by running `git fetch upstream`
3. Create a new branch from `upstream/master` by running `git checkout upstream/master` then `git checkout -b your-branch-name`
   1. Our branch naming convention is: `your_name/branch_name` (all lowercase, words separated by underscores). The branch name should be short and descriptive of the work being done on the branch.
   
**Example:** if you were working on a new navigation system using RRT and your name was "Bob" your branch name might look like: `bob/new_rrt_navigator`
4. You can now commit changes to this branch, and push them to your fork with `git push origin your_branch_name` or `git push -u`

<details>
<summary>Aside: Why should you only create branches from "upstream/master"?</summary>

Because we squash our commits when we merge Pull Requests, a new commit with a new hash will be created, containing the multiple commits from the PR branch. Because the hashes are different, git will not recognize that the squashed commit and the series of commits that are inside the squashed commit contain the same changes, which can result in conflicts.

For example, lets pretend you have _branch A_, which was originally branched from `upstream/master`. You make a few commits and open a Pull Request. While you're waiting for the Pull Request to be reviewed and merged, you create a new branch, _branch B_, from _branch A_ to get a head start on a new feature. Eventually _branch A_ gets merged into `upstream/master`. Now you want to pull the latest changes from `upstream/master` into _branch B_ to make sure you have the latest code. git will treat the squashed commit that was merged from _branch A_'s Pull Request as a new change that needs to be merged, since _branch B_ will not have a commit with the same git hash. But _branch B_ already has these changes because it was created from branch A! This will cause massive merge conflicts that are nearly impossible to resolve cleanly.

tl;dr Always create new branches from upstream/master. Do not create branches from other feature branches.

</details>

### Making Commits

We don't impose any rules for how you should be committing code, just keep the following general rules in mind:

1. Commits should represent logical steps in your workflow. Avoid making commits too large, and try keep related changes together
2. Commit messages should give a good idea of the changes made. You don't have to go in-depth with technical details, but no one will know what you've done if your commit message is "fixed broken stuff"
3. Do not commit any non-code files such as images, videos, or generated files.

### Updating Your Branch and Resolving Conflicts

As you are working on your code on your branch and making commits, you'll want to update your branch with the latest code on `upstream/master` to make sure you're working with the latest code. This is important in case someone else merged new code that affects the code you're working on.

To do this, you have 2 options: rebase or merge. [What's the difference?](https://www.atlassian.com/git/tutorials/merging-vs-rebasing). 

Merging is generally recommended, because it is easier to handle conflicts and get stuff working. To merge, simply run `git pull upstream master`.

Rebasing requires more knowledge of git and can cause crazy merge conflicts, so it isn't recommended. You can simply `git pull --rebase upstream master` to rebase your branch onto the latest `upstream/master`.

If you do rebase or merge and get conflicts, you'll need to resolve them manually. [See here for a quick tutorials on what conflicts are and how to resolve them](https://www.atlassian.com/git/tutorials/using-branches/merge-conflicts). Feel free to do this in your IDE or with whatever tool you are most comfortable with. Updating your branch often helps keep conflicts to a minimum, and when they do appear they are usually smaller. Ask for help if you're really stuck!

### Formatting Your Code

We use [clang-format](https://electronjs.org/docs/development/clang-format) to automatically format our code. Using an automatic tool helps keep things consistent across the codebase without developers having to change their personal style as they write. See the [code style guide](code-style-guide.md) for more information on exactly what it does.

To format the code, from the `Software` directory run `./formatting_scripts/fix_formatting.sh`.

We recommend running the formatting script and then committing all your changes, so that your commits can more easily pass CI.

### Pull Requests

Pull Requests give us a chance to run our automated tests and review the code before it gets merged. This helps us make sure our code on `upstream/master` always compiles and is as bug-free as possible.

The code-review process gives us a chance ask questions or suggest improvements regarding a proposed change, so that the code is of the highest possible quality before being merged. It is also a good opportunity for others on the team to see what changes are being made, even if they are not involved in the project.

The Pull Request process usually looks like the following:
 
1. Make sure all the changes you want to make are pushed to a branch on your fork of the repository
2. Make sure you have [updated your branch](#formatting-your-code) and [formatted your code](#updating-your-branch-and-resolving-conflicts). This is to help make sure CI will pass.
3. From the main page of your fork of the Software repository, click on the "code" tab and then on the "branches" tab below.
4. Find the branch you want to open a Pull Request with and click "New Pull Request"
5. Make sure the target (base-fork) is the `UBC-Thunderbots/Software` repository with branch `master`
6. Give your Pull Request a short but descriptive title (the title should reflect the changes)
7. Fill out the Pull Request template. This includes things like a description of the changes, indicating which issues the Pull Request resolves, and indicating what testing has been done.
8. Add reviewers. This should be anyone that worked with you on the changes or is working on something that will be affected by the changes. Add your team lead and a few other members. Around 3-4 reviewers is a good number, but use your best judgement. Remember, these reviews also help give other team members an idea of the changes that are being made even if they aren't working on them.
    1. At least one "Code Owner" will need to review the change in order for it to be merged
9. Click "Create Pull Request"
10. Now the code can be reviewed. Respond to feedback given by your team members and make changes as necessary by pushing additional commits to your branch.
    1. **If you are a reviewer:**
       1. Look over the code, keeping an eye out for typos, bugs, or improper [code style](code-style-guide.md)
       2. If you are having trouble understanding what a certain part of the code is doing, that's a great place to suggest adding additional comments!
       3. Remember you are critiquing someone's work. Give useful, constructive feedback and justify your thoughts, and don't be mean or degrading. Try to provide a suggested solution where possible.
       4. During re-reviews (Pull Requests typically involve several rounds of changes and review), **it is your responsibility to check that previously requested changes were made and mark the relevant discussions as "resolved"**. "Unresolved" discussions make a great checklist of things to check during a re-review.
       5. Mark the Pull Request as "Approved" when you think it looks good
    2. **If you are the recipient of the review (the PR creator):**
       1. **Make sure to reply to the PR comments as you address / fix issues**. This helps the reviewers know you have made a change without having to go check the code diffs to see if you made a change.
          1. Eg. Reply with "done" or "fixed" to comments as you address them
          2. Leave comments unresolved, let the reviewer resolve them.
       2. Don't be afraid to ask for clarification regarding changes or suggest alternatives if you don't agree with what was suggested. The reviewers and reviewee should work together to come up with the best solution.
       3. **Do not resolve conversations as you address them** (but make sure to leave a comment as mentioned above). That is the responsibility of the reviewers.
       4. Once you have addressed all the comments, re-request review from reviewers.
11. Make sure our automated tests with Github Actions are passing. There will be an indicator near the bottom of the Pull Request. If something fails, you can click on the links provided to get more information and debug the problems. More than likely, you'll just need to re-run clang-format on the code.
12. Once your Pull Request has been approved and the automated tests pass, you can merge the code. There will be a big 'merge" button at the bottom of the Pull Request with several options to choose from
    1. We only allow "Squash and merge". This is because it keep the commit history on `upstream/master` shorter and cleaner, without losing any context from the commit messages (since they are combined in the squashed commit. A squashed commit also makes it easier to revert and entire change/feature, rather than having to "know" the range of commits to revert.
13. That's it, your changes have been merged! You will be given the option to delete your remote branch. but are not required to do so. We recommend it since it will keep your fork cleaner, but you can do whatever you like.

*Remember, code reviews can be tough. As a reviewer, it can be very tricky to give useful constructive criticism without coming off as condescending or degrading (emotions are hard to express through text!). As the recipient of a code review, it might feel like you are being criticized too harshly and that your hard work is being attacked. Remember that these are your teammates, who are not trying to arbitrarily devalue your contributions but are trying to help make the code as good as possible, for the good of the team.*

### Reviewing Pull Requests

When reviewing Pull Requests, it can be really difficult to phrase comments in a way that doesn't come across as aggressive or mean. That said, it's really important that we strive to keep Pull Requests friendly and open, both for the health of everyone involved, and the effectiveness of the code review process. Here are two links that everyone reviewing a pull request should _thoroughly_ read before doing reviews:

[https://mtlynch.io/human-code-reviews-1/](https://mtlynch.io/human-code-reviews-1/) 

[https://mtlynch.io/human-code-reviews-2/](https://mtlynch.io/human-code-reviews-2/)


## Example Workflow

We find our workflow is best explained by walking through an example. We're assuming you have already cloned the repository and set up your git remotes. If not, check out the [Getting the Code](#getting-the-code) instructions first and then come back.

This example incorporates information from the previous sections on [Issue and Project Tracking](#issue-and-project-tracking), and [the Git Workflow](#git-workflow). Make sure you have read those sections first. This example skips over some of the smaller details.
  
We are also assuming all the work done here is in your fork of the repository.

Let's pretend our goalie strategy isn't that great. You have noticed that and suggested we improve it. Here's what your workflow would likely look like, from start to finish. We will pretend your name is Bob.

1. Create a new Issue for the goalie strategy if it doesn't already exist
   1. Let's pretend this is `Issue #23`
   2. Add an estimated Difficulty tag to this issue
2. Create a new branch from `upstream/master`, called `bob/create_new_goalie_strategy`
   1. `git fetch upstream`
   2. `git checkout -b bob/create_new_goalie_strategy upstream/master`
3. Make your changes
   1. As you make changes and come across new information / challenges, it is good to update the Issue you are working on to document these new changes or requirements. Updating our progress on the ticket also helps other know how your work is going.
      1. `git commit -m "Improved the goalie's positioning during corner kicks, to block shots near the edge of the net"`
   2. Don't forget to push your changes to the branch on your fork occasionally, so you don't lose your work if something happens to your computer (it's happened to our team before)
4. Open a Pull Request to the master branch of the main Software repository. This will be a request to merge the branch `bob/create_new_goalie_strategy` from your fork, to the `master` branch of the main `Software` repository.
   1. The description should include `resolves #23`, so we know this Pull Request resolved your ticket
5. Discuss the changes with your reviewers and update the Pull Request by pushing additional commits to your branch
6. Once the Pull Request is approved and all CI checks pass, "Squash and merge" the changes
7. **Optional:** Delete your remote branch
8. Make sure your issue is marked as resolved. If you remembered to include `resolves #23` in your Pull Request description, this should have been done automatically for you, but it's good to double check.
9. Congratulations, you've now made changes to our main codebase!

## Testing

Testing is an integral part of our development process. If you are writing basically **any** code, it should be tested. If you feel like you can't test your piece of code, it's likely because it was written in a way that makes it difficult to test (which is a strong indicator for other problems, such as too much [coupling](https://en.wikipedia.org/wiki/Coupling_%28computer_programming%29)). _(An exception to this rule is any code that talks **directly** with hardware)_. For some examples of what happens when people don't test their code enough, see [here](http://outfresh.com/knowledge-base/6-famous-software-disasters-due-lack-testing/). How to run tests is explained in the [Building and Running Code](#building-and-running-the-code) section. 

