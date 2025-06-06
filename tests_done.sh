# Run on Ubu24
sudo docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v $HOME/.cache:/root/.cache --name test test:latest

# Run on Mac
sudo docker run --rm -it -e DISPLAY=host.docker.internal:0 --mount type=bind,source=$HOME/Programs/Thunderbot/.cache,target=/root/.cache --mount type=bind,source=$HOME/Programs/Thunderbot/Software,target=/Software -v /tmp/.X11-unix:/tmp/.X11-unix --privileged --name testv1 --net=host test:0.1

# Run bazel inside docker
bazel run //software/thunderscope:thunderscope_main


# Clearing:
sudo docker system prune

# Building:
sudo docker build -t test:latest .


# To fix -> MESA: error: ZINK: failed to choose pdev & glx: failed to create drisw screen

sudo add-apt-repository ppa:kisak/kisak-mesa -y && sudo apt update && sudo apt upgrade -y

export QMLSCENE_DEVICE=softwarecontext
export QT_OPENGL=software
export LIBGL_ALWAYS_SOFTWARE=1

# To fix -> No matching fbConfigs or visuals found
export LIBGL_ALWAYS_INDIRECT=1

apt-get update && apt-get install -y     libglvnd-dev    libxi-dev     libxrandr-dev     libx11-dev     libfontconfig1-dev     libfreetype6-dev     qt5-qmake qtbase5-dev qtchooser qtbase5-dev-tools     xvfb     mesa-utils     gdb

# opengl testing command
glxinfo
glxgears

# Enable IGLX on host
xhost +
defaults write org.xquartz.X11 enable_iglx -bool true
