# Docker file for virtual robocup 2021

FROM robocupssl/ubuntu-vnc:latest

# Fetches the repo from the repository and
# places it in a folder called "Software" in the
# containers home directory. Then runs setup_software.sh
# and updates the alternatives to make sure correct
# version of gcc is used.

USER root
RUN apt-get -qq update && apt-get -qq install -y git && apt-get -qq install sudo
RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata
RUN git clone https://github.com/UBC-Thunderbots/Software.git /Software
RUN /Software/environment_setup/setup_software.sh
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
USER default
