# ベースイメージ（GPUサポートが必要な場合はnvidia/opengl:1.0-glvnd-runtime-ubuntu20.04を推奨）
FROM osrf/ros:noetic-desktop-full

# 環境設定
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    DISPLAY=:1 \
    VNC_PORT=5901 \
    NOVNC_PORT=8080

# 基本パッケージ + GUI環境 + 開発ツール
RUN apt-get update && \
    apt-get install -y \
    sudo git nano curl wget \
    xfce4 xfce4-goodies x11vnc xvfb \
    websockify novnc python3-pip \
    python3-tk python3-dev \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    supervisor unzip \
    && rm -rf /var/lib/apt/lists/*

# Python依存関係
RUN pip3 install --upgrade pip && \
    pip3 install \
    neat-python \
    numpy matplotlib \
    pybullet==3.2.5 \
    opencv-python-headless

# VNC設定（パスワード: 1234）
RUN mkdir -p /root/.vnc && \
    x11vnc -storepasswd 1234 /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd

# マウントポイント作成
RUN mkdir -p /workspace/catkin_ws/src && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Supervisor設定
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# ポート公開
EXPOSE $VNC_PORT $NOVNC_PORT

# ワークディレクトリ
WORKDIR /workspace

# 起動スクリプト
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
