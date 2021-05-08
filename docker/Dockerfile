# Docker file for virtual robocup 2021

FROM robocupssl/ubuntu-vnc:latest

# Creates a directory called scripts, copies
# getRepo script and runs it to fetch the git
# repository. Once the repository is fetched,
# it runs setup_software
USER root
RUN apt-get -qq update && apt-get -qq install -y git && apt-get -qq install sudo
RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata
RUN git clone https://github.com/UBC-Thunderbots/Software.git /Software
RUN /Software/environment_setup/setup_software.sh
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
USER default
