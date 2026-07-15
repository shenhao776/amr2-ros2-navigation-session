FROM osrf/ros:humble-desktop-full

# set zone to Asia/Tokyo
ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# apt packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    vim \
    iputils-ping \
    net-tools \
    zsh \
    wget \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    libpcap-dev \
    protobuf-compiler \
    libopencv-dev \
    # ROS2 依赖包 (humble 发行版固定, build 号由 apt 自动解析)
    ros-humble-tf-transformations \
    ros-humble-rqt-tf-tree \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rqt-image-view \
    ros-humble-image-view \
    ros-humble-sophus \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-py \
    ros-humble-rosidl-runtime-py \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    # turtlebot3 / realsense
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-bringup \
    ros-humble-librealsense2 \
    ros-humble-realsense2-camera \
    && \
    install -m 0755 -d /etc/apt/keyrings && \
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg && \
    chmod a+r /etc/apt/keyrings/docker.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null && \
    apt-get update && apt-get install -y docker-ce-cli && \
    rm -rf /var/lib/apt/lists/*

# install Python
RUN pip3 install --no-cache-dir --upgrade pip
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# compile Livox-SDK2 and setup zsh
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    git checkout v1.2.5 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    cd ../.. && rm -rf Livox-SDK2 && \
    # ZSH
    wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh && \
    sh install.sh && rm install.sh && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    # Comment out the conflicting line in rosidl-argcomplete.zsh
    sed -i 's/^autoload -U +X compinit && compinit/# &/' /opt/ros/humble/share/rosidl_cli/environment/rosidl-argcomplete.zsh && \
    # Modify .zshrc plugins line
    sed -i 's/^plugins=(git)$/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' ~/.zshrc && \
    sed -i 's/^ZSH_THEME="robbyrussell"$/ZSH_THEME="ys"/' ~/.zshrc

# 5. workspace
WORKDIR /root/shared_files
# --- START: ADDED FOR Livox Driver ---
# Clone and build the livox_ros_driver2. This is a third-party dependency.
RUN /bin/zsh -c "source /opt/ros/humble/setup.zsh && \
    mkdir -p /root/thirdparty_ws/src && \
    cd /root/thirdparty_ws/src && \
    git clone https://github.com/shenhao776/livox_ros_driver2.git && \
    cd livox_ros_driver2 && \
    git checkout a5b91b6b8a00e610002f53b99562ff7df0c7f9e6 && \
    ./build.sh humble"

RUN echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.zshrc && \
    echo 'source /opt/ros/humble/setup.zsh' >> ~/.zshrc && \
    echo 'source /root/thirdparty_ws/install/setup.zsh' >> ~/.zshrc && \
    echo 'source ~/shared_files/amr2-ros2-navigation-session/ros2_ws/install/setup.zsh' >> ~/.zshrc && \
    echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.zshrc

CMD ["zsh"]
