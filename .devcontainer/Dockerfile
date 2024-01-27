FROM osrf/ros:noetic-desktop-full

ARG USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME
RUN useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ARG WORKSPACE=dev_container
WORKDIR /workspace/$WORKSPACE

# Use all colors for the TTY.
ENV TERM=xterm-256color

RUN apt-get update && apt-get install -y \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-global-planner \
    ros-noetic-teb-local-planner

RUN apt-get install -y \
    curl \
    git \
    nano \
    wget \
    zsh

# Install Oh My Zsh.
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -x

RUN echo "export DISABLE_AUTO_TITLE=true" >> ${HOME}/.zshrc
RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> ${HOME}/.zshrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.zsh" >> ${HOME}/.zshrc

# Setup zsh shell as default shell.
RUN chsh -s $(which zsh)

USER ${USERNAME}

CMD ["/usr/bin/env", "zsh"]