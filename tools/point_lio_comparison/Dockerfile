FROM ros:noetic-ros-base-focal
COPY download_debs.py /tmp/download_debs.py
RUN apt-get update && \
    apt-get --print-uris -y --no-install-recommends install \
      build-essential python3-dev python3-numpy libgoogle-glog-dev \
      ros-noetic-pcl-ros ros-noetic-eigen-conversions > /tmp/package-uris && \
    python3 /tmp/download_debs.py /tmp/package-uris && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential python3-dev python3-numpy libgoogle-glog-dev \
      ros-noetic-pcl-ros ros-noetic-eigen-conversions && rm -rf /var/lib/apt/lists/*
