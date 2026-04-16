FROM ros:humble-ros-base

# 设置环境变量，避免交互模式安装卡顿
ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO=humble


# 使用清华大学的apt源
RUN sed -i 's/http:\/\/archive.ubuntu.com/http:\/\/mirrors.tuna.tsinghua.edu.cn\/ubuntu/g' /etc/apt/sources.list


RUN apt update && apt install -y --no-install-recommends \
    # 基础依赖工具
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    # C++/CMake 编译环境
    build-essential \
    gcc \
    g++ \
    cmake \
    # 常用工具
    python3-pip \
    wget \
    htop \
    vim \
    unzip \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-black \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    # 系统库依赖（对应报错的包）
    libpcl-dev \
    libapr1-dev \
    libaprutil1-dev \
    # Nav2 相关包
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # grid_map 相关包
    ros-humble-grid-map-core \
    ros-humble-grid-map-ros \
    ros-humble-grid-map-rviz-plugin \
    ros-humble-grid-map-filters \
    ros-humble-grid-map-loader \
    # ROS 构建工具
    python3-rosdep \
    python3-colcon-common-extensions \
    # small_gicp 依赖
    libeigen3-dev \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# 使用pip的--index-url选项设置pip源
RUN pip install --upgrade pip
RUN pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple


RUN pip install xmacro gdown
#使用 gitclone.com 代理
#RUN git config --global url."https://gitclone.com/github.com/".insteadOf "https://github.com/"
RUN git config --global http.proxy http://192.168.2.139:7897 && \
    git config --global https.proxy http://192.168.2.139:7897

# Install small_gicp
RUN mkdir -p /tmp/small_gicp && \
    cd /tmp && \
    git clone https://github.com/koide3/small_gicp.git && \
    cd small_gicp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/small_gicp


RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

###################################
############## pb_sentry ##############
###################################

# 克隆 pb_sentry 到 /root/pb_sentry
RUN git clone https://github.com/YD07-xyt/pb_sentry.git /root/pb_sentry
# COPY src /root/pb_sentry/src
# COPY sh  /root/pb_sentry/sh


# 进入 pb_sentry 目录后执行 rosdep install
WORKDIR /root/pb_sentry
RUN rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 编译 pb_sentry 项目
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 自动加载 pb_sentry 编译后的环境
RUN echo "source /root/pb_sentry/install/setup.bash" >> /root/.bashrc

# 恢复交互模式
ENV DEBIAN_FRONTEND=dialog


CMD ["/bin/bash"]