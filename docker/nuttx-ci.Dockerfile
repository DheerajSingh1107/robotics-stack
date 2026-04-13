FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    bison \
    build-essential \
    flex \
    genromfs \
    gperf \
    libncurses5-dev \
    libncursesw5-dev \
    pkg-config \
    python3 \
    python3-pip \
    git \
    kconfig-frontends \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
