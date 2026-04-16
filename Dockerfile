FROM ros:humble-ros-base

# 设置环境变量，避免交互模式安装卡顿
ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO=humble

# 使用清华大学的apt源
RUN sed -i 's/http:\/\/archive.ubuntu.com/http:\/\/mirrors.tuna.tsinghua.edu.cn\/ubuntu/g' /etc/apt/sources.list

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    gdb \
    valgrind \
    git \
    libssl-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

# RUN pip install --upgrade pip
# RUN pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

RUN git config --global http.proxy http://192.168.1.251:7897 && \
    git config --global https.proxy http://192.168.1.251:7897

ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} xyt && useradd -m -s /bin/bash -u ${USER_ID} -g ${GROUP_ID} xyt
USER xyt

WORKDIR /code

ENV DEBIAN_FRONTEND=dialog

CMD ["/bin/bash"]