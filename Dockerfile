FROM ubuntu:24.04

RUN apt-get update && apt-get install -y software-properties-common wget fzf

RUN add-apt-repository ppa:kisak/kisak-mesa -y && apt update && apt upgrade -y

# RUN apt-get update && apt-get install -y \
#     mesa-utils \
#     libgl1-mesa-dri \
#     libgl1-mesa-glx \
#     && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1 \
    libglx-mesa0

WORKDIR /Software
COPY . .

# Redirecting sudo to a dummy command
RUN echo "#!/bin/bash\n\$@" > /usr/bin/sudo
RUN chmod +x /usr/bin/sudo

# Setup the sofrware environment
RUN chmod +x ./environment_setup/setup_software.sh && ./environment_setup/setup_software.sh

# Build bazel cache
# RUN ["bazel", "build", "//software/thunderscope:thunderscope_main"]

ENV QMLSCENE_DEVICE=softwarecontext
ENV QT_OPENGL=software
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV LIBGL_ALWAYS_INDIRECT=1


WORKDIR /Software/src
CMD ["/bin/bash"]

