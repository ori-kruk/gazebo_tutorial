FROM ubuntu:22.04 as base


ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* 



RUN apt-get update \
  && apt-get install -y \
  build-essential \
  cmake \
  git \
  gnupg \
  lsb-release \
  wget \
  vim \
  curl \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* 

RUN curl -sSL http://get.gazebosim.org | sh

