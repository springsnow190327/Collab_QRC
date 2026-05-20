#!/bin/bash
set -e

# Download and install ccache
wget https://github.com/ccache/ccache/releases/download/v4.10.2/ccache-4.10.2-linux-x86_64.tar.xz && \
    tar -xvf ccache-4.10.2-linux-x86_64.tar.xz  && \
    cd ccache-4.10.2-linux-x86_64 && make install && \
    ln -s ccache /usr/local/bin/cc && \
    ln -s ccache /usr/local/bin/c++ && \
    ln -s ccache /usr/local/bin/nvcc && \
    echo 'ccache --max-size 10GB' | tee  --append /etc/bash.bashrc
