# base image
FROM ubuntu

# Creates a directory called scripts, copies
# getRepo script and runs it to fetch the git
# repository. Once the repository is fetched,
# it runs setup_software
RUN apt-get -qq update && apt-get -qq install -y git && apt-get -qq install sudo
RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata
RUN mkdir -p /scripts
COPY setup_repo.sh /scripts
RUN chmod +x /scripts/setup_repo.sh
RUN /scripts/setup_repo.sh
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7

# Temporary entrance to the docker container
# with tail -f /dev/null
# run following command
# docker run -it <container name> /bin/bash
# to open a shell in the container
CMD [ "tail","-f","/dev/null" ]
