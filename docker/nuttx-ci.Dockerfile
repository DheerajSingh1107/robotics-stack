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
    wget \
    automake \
    autoconf \
    libtool \
    m4 \
    perl \
    vim-common \
    && rm -rf /var/lib/apt/lists/*

# Fix versioned autotools names
RUN ln -s /usr/bin/aclocal /usr/bin/aclocal-1.15 && \
    ln -s /usr/bin/automake /usr/bin/automake-1.15

# Install kconfig-frontends
RUN git clone https://bitbucket.org/nuttx/tools.git /tmp/nuttx-tools && \
    cd /tmp/nuttx-tools/kconfig-frontends && \
    ./configure --enable-mconf --enable-nconf --disable-gconf --disable-qconf && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/nuttx-tools

WORKDIR /workspace
