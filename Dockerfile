FROM ros:dashing-ros-base-bionic

# install required packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    curl ros-dashing-ament-cmake qttools5-dev-tools \
    libqt5svg5-dev qtmultimedia5-dev qtcreator \
    qtcreator-data vim qml freeglut3-dev \
    python3-pip ca-certificates gnupg lsb-core wget \
    autoconf automake bison bzip2 flex gdb-multiarch gperf \
    libncurses-dev libtool pkg-config vim-common python-jinja2 \
    python-empy python-toml python-numpy python-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U pip && python3 -m pip install -U setuptools
RUN python -m pip install -U setuptools && python -m pip install -U pyyaml
RUN pip3 install numpy>=1.15.4
RUN pip3 install argparse cerberus empy jinja2  \
packaging pandas psutil pygments pyros-genmsg pyserial pyulog  \
pyyaml six toml wheel opencv-python imutils shapely ipdb geopy

RUN mkdir /px4_setup && cd /px4_setup && curl https://raw.githubusercontent.com/PX4/PX4-Autopilot/master/Tools/setup/requirements.txt -o /px4_setup/requirements.txt && curl https://raw.githubusercontent.com/PX4/PX4-Autopilot/aad7e88af2ceedfdfa639ae6c58c40aa4cb03119/Tools/setup/ubuntu.sh -o /px4_setup/ubuntu.sh
RUN cd /px4_setup && /bin/bash -c "source ubuntu.sh"

RUN echo "" && echo "export LANG=C.UTF-8" >> ~/.bashrc && echo "export LC_ALL=C.UTF-8" >> ~/.bashrc && echo "export PATH=$PATH:/opt/gcc-arm-none-eabi-7-2017-q4-major/bin" >> ~/.bashrc && echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
