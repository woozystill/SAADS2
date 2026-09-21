FROM nvcr.io/nvidia/pytorch:25.06-py3-igpu

ENV DEBIAN_FRONTEND=noninteractive

# Basic runtime packages
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Keep NVIDIA's tested NumPy/OpenCV/PyTorch environment stable.
RUN pip install --no-cache-dir \
    "numpy==1.26.4" \
    "opencv-python-headless==4.11.0.86"

# Install Ultralytics without replacing NVIDIA's PyTorch stack.
RUN pip install --no-cache-dir ultralytics --no-deps

WORKDIR /workspace
