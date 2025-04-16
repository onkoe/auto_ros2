# we'll use Ubuntu 22.04, the only version of Ubuntu supported by ROS 2 Humble
FROM ubuntu:22.04

################
# environment variables
################
#
# ...keep aptitude quiet
ENV DEBIAN_FRONTEND=NONINTERACTIVE
#
# ...vnc stuff
ENV VNC_PORT=5901
ENV NOVNC_PORT=5801
ENV SSHD_PORT=22
#
# ...user/password
ENV USER=soro
ENV PASS=NotIan!
#
# ...language
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
#
# ...ros2 version
ENV ROS_DISTRO=humble

################
# setup stuff
################

# use bash for everything while installing.
#
# this is important for ros 2...
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# install the shell
RUN apt-get update && apt-get install -y --no-install-recommends fish curl && rm -rf /var/lib/apt/lists/*

# use the correct timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# set up ROS 2 packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends dirmngr gnupg2 \
    && rm -rf /var/lib/apt/lists/*
RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2-latest.list

# install a few dependencies before using `rosdep`...
#
# this includes ros 2 humble core!
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
    build-essential git python3-colcon-common-extensions python3-colcon-mixin python3-rosdep python3-vcstool \
    ros-humble-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*


# use `rosdep` to install (a lot) of humble pkgs
RUN . /opt/ros/humble/setup.bash \
    && rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO

# update colon mixins
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update \
    && colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
    && colcon metadata update

# install ros 2 humble base + simulation pkgs
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-ros-base=0.10.0-1* ros-humble-simulation=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# default user
#
# ...make the user
RUN useradd \
    $USER \
    --shell /usr/bin/fish \
    --create-home \
    --groups sudo
#
# ...assign user a _very good_ password
RUN echo "$USER:$PASS" | chpasswd
#
# ...make them a `~/bin`
RUN mkdir -p /home/$USER/bin
#
# ...give them ownership of their home dir
RUN chown -R $USER /home/$USER/

# grab `bass` for `fish`
RUN su - $USER -c "fish -c 'curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && \
    fisher install jorgebucaran/fisher && \
    fisher install edc/bass && \
    fish_add_path /home/$USER/bin'"

# turn on ubuntu universe repos (required for common ros 2/etc. tools)
RUN apt-get update \
    && apt-get install -y software-properties-common \
    && add-apt-repository -y universe \
    && rm -rf /var/lib/apt/lists/*

# set up simulator repos and install gazebo fortress
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# install the simulator
RUN apt-get update \
    && apt-get install -y \
    ros-humble-ros-ign-bridge ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz \
    ros-humble-ros-gz ros-humble-geographic-msgs ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# grab rust and uv
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs
COPY --from=ghcr.io/astral-sh/uv:0.6.3 /uv /uvx /bin/

# grab turbovnc, a crazy-fast vnc server
#
# ...signing key
RUN curl -Ls https://packagecloud.io/dcommander/turbovnc/gpgkey | \
    gpg --dearmor > /etc/apt/trusted.gpg.d/TurboVNC.gpg

#
# ...repo
COPY sysroot/etc/apt/sources.list.d/TurboVNC.list /etc/apt/sources.list.d/TurboVNC.list

# install all the gui stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    nginx supervisor websockify novnc \
    turbovnc xserver-xorg x11-xserver-utils \
    dbus-x11 x11-utils alsa-utils pulseaudio mesa-utils libgl1-mesa-dri \
    xorg xserver-xorg-video-dummy \
    xfce4 xfwm4 xfce4-panel xfce4-settings xfce4-clipman xfce4-session \
    thunar mousepad xfce4-terminal xfce4-taskmanager \
    wget htop ripgrep unzip \
    sudo net-tools iputils-ping zenity xz-utils xinput xterm \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# theming
RUN apt-get update \
    && apt-get install -y --no-install-recommends xfce4-whiskermenu-plugin adwaita-icon-theme-full fonts-inter gtk2-engines adwaita-icon-theme \
    && rm -rf /var/lib/apt/lists/*

# also grab kitty, a terminal emulator
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common \
    && apt-get update \
    && apt-get install -y --no-install-recommends kitty \
    && rm -rf /var/lib/apt/lists/*

# other userspace stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim psmisc \
    && rm -rf /var/lib/apt/lists/*

# install UTF-8 locale stuff, if needed
RUN apt-get update \
    && apt-get install locales -y \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# install rust
RUN su soro -c "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y"

################
# finishing touches
################

# prep for sshd
RUN mkdir /var/run/sshd

# sysroot
COPY sysroot/ /

# make a password file for vnc
RUN mkdir -p /home/$USER/.vnc
RUN touch /home/$USER/.vnc/passwd && chmod 600 /home/$USER/.vnc/passwd
RUN echo "$PASS" | /opt/TurboVNC/bin/vncpasswd -f >> /home/$USER/.vnc/passwd
RUN chown -R $USER /home/$USER/

# download + copy the theme over
RUN curl --location --output /tmp/adw-gtk3.tar.xz \
    https://github.com/lassekongo83/adw-gtk3/releases/download/v5.6/adw-gtk3v5.6.tar.xz \
    && xz --decompress /tmp/adw-gtk3.tar.xz \
    && tar -xf /tmp/adw-gtk3.tar --directory=/home/$USER/.local/share/themes/ \
    && rm -rf /tmp/adw-gtk3.tar

# do the same for the xfwm4 theme
RUN git clone --depth 1 --quiet \
    https://github.com/onkoe/adw-xfwm4.git \
    /tmp/adw-xfwm4 \
    && mv /tmp/adw-xfwm4/themes/adw-gtk3-dark/xfwm4 /home/soro/.local/share/themes/adw-gtk3-dark/ \
    && mv /tmp/adw-xfwm4/themes/adw-gtk3/xfwm4 /home/soro/.local/share/themes/adw-gtk3/ \
    && rm -rfd /tmp/adw-xfwm4

# install the `iosevka` font (not packaged in ubuntu 22.04)
RUN wget -O /tmp/fonts-iosevka.zip \
    https://github.com/be5invis/Iosevka/releases/download/v33.2.0/PkgTTC-Iosevka-33.2.0.zip \
    && unzip /tmp/fonts-iosevka.zip -d /usr/share/fonts/truetype/iosevka \
    && rm /tmp/fonts-iosevka.zip \
    && sudo fc-cache -fv

# expose ports to host
EXPOSE $VNC_PORT
EXPOSE $NOVNC_PORT
EXPOSE $SSHD_PORT

# immediately run supervisord
CMD ["/usr/bin/env", "supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf", "--logfile", "/tmp/supervisord.log"]
