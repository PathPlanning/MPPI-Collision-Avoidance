FROM ubuntu:22.04


# Ubuntu setup
RUN apt-get update && apt-get install -y --no-install-recommends \
    dialog apt-utils zsh sudo && \
    echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    apt-get install -y -q && \
    rm -rf /var/lib/apt/lists/*


# Create User
ENV NEWUSER="user"
RUN adduser --home /home/${NEWUSER} --shell /bin/zsh --gecos '' --disabled-password ${NEWUSER}
RUN usermod -a -G root,sudo,video,tty,dialout ${NEWUSER} 
RUN echo "${NEWUSER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${NEWUSER}





# Python
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        python3-tk \
        python3-pip \
        cmake && \
    rm -rf /var/lib/apt/lists/*

# Python Packages
RUN pip3 install --upgrade pip && \
    pip3 install \
        numpy \
        scipy \
        lxml \
        argparse \
        pandas \
        numba \
        matplotlib \
        seaborn \
        tqdm \
        yaml-1.3 \
        pynput \
        vidgear \
        opencv-python \
        opencv-contrib-python \
        jinja2 \
        filterpy


# Utils 
RUN apt-get update && \
    apt-get install -y \
        tmux \
        ranger \
        wget \
        neovim \
        nano \
        htop \
        iputils-ping && \
        rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
      apt-get upgrade -y && \
        rm -rf /var/lib/apt/lists/*

# Scripts
USER $NEWUSER

# RUN -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.0/zsh-in-docker.sh)" -- \
# 	-p git \
# 	-p https://github.com/zsh-users/zsh-autosuggestions \
# 	-p https://github.com/zsh-users/zsh-completions \
# 	-p https://github.com/zsh-users/zsh-syntax-highlighting 


RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.0/zsh-in-docker.sh)" -- \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting 

RUN echo "unsetopt share_history" >> ~/.zsh_profile
RUN echo "unsetopt share_history" >> ~/.zshrc

RUN mkdir -p ~/mppi-ca/src
RUN mkdir -p ~/mppi-ca/experiments


WORKDIR /home/$NEWUSER
ENTRYPOINT []