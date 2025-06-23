# DiaROS Docker Image - Lightweight ROS2 Humble with Speech Dialog System
FROM osrf/ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install system dependencies and ROS2 tools
RUN apt-get update && apt-get install -y \
    # Basic tools
    curl \
    wget \
    git \
    vim \
    sudo \
    # Build tools
    build-essential \
    cmake \
    # Python dependencies
    python3-pip \
    python3-dev \
    python3-setuptools \
    # Audio processing dependencies
    portaudio19-dev \
    libasound2-dev \
    libaubio-dev \
    libsndfile1-dev \
    # GUI dependencies for rqt
    python3-pyqt5 \
    pyqt5-dev-tools \
    # Locale support for Japanese
    locales \
    && locale-gen ja_JP.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages for monitoring and debugging
RUN apt-get update && apt-get install -y \
    # Basic rqt tools
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-rqt-topic \
    ros-${ROS_DISTRO}-rqt-console \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    # Data recording and playback tools
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-rqt-bag \
    # Additional ROS2 tools
    ros-${ROS_DISTRO}-ros2topic \
    ros-${ROS_DISTRO}-ros2node \
    ros-${ROS_DISTRO}-ros2service \
    ros-${ROS_DISTRO}-ros2action \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python packages
RUN python3 -m pip install --upgrade pip setuptools wheel

# Install Python packages for speech processing
RUN pip3 install --no-cache-dir \
    # Audio processing
    pyaudio \
    aubio \
    # Speech recognition
    google-cloud-speech \
    SpeechRecognition \
    # Machine learning (CPU version to save space)
    torch --index-url https://download.pytorch.org/whl/cpu \
    transformers \
    # Other utilities
    numpy \
    scipy \
    requests \
    pyyaml

# Install VOICEVOX Core (lightweight version without GUI)
RUN mkdir -p /opt/voicevox && \
    cd /opt/voicevox && \
    wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.4/voicevox_core-0.14.4+cpu-cp38-abi3-linux_x86_64.whl && \
    pip3 install voicevox_core-0.14.4+cpu-cp38-abi3-linux_x86_64.whl && \
    rm voicevox_core-0.14.4+cpu-cp38-abi3-linux_x86_64.whl

# Create workspace directory
RUN mkdir -p /workspace

# Set up ROS2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /workspace

# Copy entrypoint script
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]