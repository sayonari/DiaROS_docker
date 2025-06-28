# DiaROS Docker Image - Lightweight ROS2 Humble with Speech Dialog System
FROM ros:humble-ros-base

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
    python3-wheel \
    python3-numpy \
    python3-numpy-dev \
    # Audio processing dependencies
    portaudio19-dev \
    libasound2-dev \
    libasound2-plugins \
    libaubio-dev \
    libsndfile1-dev \
    ffmpeg \
    pulseaudio \
    pulseaudio-utils \
    sox \
    # GStreamer for audio playback
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-python3-plugin-loader \
    python3-gst-1.0 \
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

# Upgrade pip and install Python packages with compatible setuptools version
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install setuptools==65.5.0 wheel && \
    python3 -m pip install --force-reinstall "numpy==1.24.3"

# Install Python packages for speech processing
# Install pyaudio with its dependencies
RUN pip3 install --no-cache-dir pyaudio

# Install other Python packages
RUN pip3 install --no-cache-dir \
    # Speech recognition
    SpeechRecognition \
    # Audio processing
    playsound \
    webrtcvad \
    librosa \
    soundfile \
    pydub \
    # Other utilities (scipy needs compatible numpy version)
    "scipy==1.10.1" \
    requests \
    pyyaml \
    huggingface-hub

# Install aubio Python bindings with numpy 1.x compatibility
# First ensure numpy is at correct version, then install aubio
RUN python3 -m pip install --no-cache-dir --no-deps aubio==0.4.9 && \
    python3 -c "import aubio; print('aubio successfully installed with numpy', aubio.__version__)" || \
    echo "Warning: aubio installation failed, using system library only"

# Install PyTorch and transformers separately to handle index URL
# Use ignore-installed to bypass system sympy
RUN pip3 install --no-cache-dir --ignore-installed torch --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --no-cache-dir transformers sentencepiece fugashi unidic-lite

# Install VOICEVOX Core (lightweight version without GUI)
# Detect architecture and download appropriate wheel
RUN mkdir -p /opt/voicevox && \
    cd /opt/voicevox && \
    ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.4/voicevox_core-0.14.4+cpu-cp38-abi3-linux_x86_64.whl && \
        pip3 install voicevox_core-0.14.4+cpu-cp38-abi3-linux_x86_64.whl && \
        rm voicevox_core-0.14.4+cpu-cp38-abi3-linux_x86_64.whl; \
    elif [ "$ARCH" = "aarch64" ]; then \
        echo "VOICEVOX Core is not available for ARM64 architecture. Skipping installation."; \
    else \
        echo "Unsupported architecture: $ARCH"; \
    fi

# Create workspace directory
RUN mkdir -p /workspace

# Set up ROS2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /workspace

# Copy scripts
COPY scripts/entrypoint.sh /entrypoint.sh
COPY scripts/start_diaros.sh /workspace/scripts/start_diaros.sh
RUN chmod +x /entrypoint.sh /workspace/scripts/start_diaros.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]