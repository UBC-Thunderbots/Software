# base image
FROM ubuntu

# Creates a directory called scripts, copies
# getRepo script and runs it to fetch the git
# repository. Once the repository is fetched,
# it runs setup_software
RUN apt-get -qq update && apt-get -qq install -y git
RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata
RUN mkdir -p /scripts
COPY getRepo.sh /scripts
COPY setup_software.sh /scripts
RUN chmod +x /scripts/getRepo.sh
RUN chmod +x /scripts/setup_software.sh
RUN /scripts/getRepo.sh
RUN /scripts/setup_software.sh

# Temporary entrance to the docker container
# with tail -f /dev/null
# run following command
# docker exec -it <container name> /bin/bash
# to open a shell in the container
CMD [ "tail","-f","/dev/null" ]
