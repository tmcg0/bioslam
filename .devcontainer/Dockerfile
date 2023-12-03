# --------------------------------------------------------------------------- #
#      bioslam (latest) development Dockerfile for Ubuntu 20.04 base image          
# with dependencies:
#   boost: 1.71.0 (https://packages.ubuntu.com/focal/libboost1.71-all-dev)
#   TBB: 2020.1 (https://packages.ubuntu.com/focal/libtbb-dev)
#   HDF5 (serial): 1.10.4 (https://packages.ubuntu.com/focal/libhdf5-dev)
#   Eigen: 3.3.9 (https://gitlab.com/libeigen/eigen)
#   GTSAM: 4.0.3 (https://github.com/borglab/gtsam/releases/4.0.3)
#   HighFive: 2.8.0 (https://github.com/BlueBrain/HighFive/releases/v2.8.0)
# build environment:
#   git: 2.25.1 (https://packages.ubuntu.com/focal/git)
#   cmake: 3.16.3 (https://packages.ubuntu.com/focal/cmake)
#   g++: 9.3.0 (from build-essential: https://packages.ubuntu.com/focal/build-essential)
#   make: 4.2.1 (from build-essential: https://packages.ubuntu.com/focal/build-essential)
# notes:
# - only builds the C++ library and executables, no Python/MATLAB wrappers
# - internet connection required to download dependencies
# --------------------------------------------------------------------------- #

FROM ubuntu:20.04

LABEL maintainer="Tim McGrath <t.mike.mcgrath@gmail.com>"

ARG N_JOBS=4

ENV DEBIAN_FRONTEND noninteractive

# install package dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    build-essential \
    git \
    cmake \
    libhdf5-dev \
    libboost1.71-all-dev \
    libtbb-dev \
    && apt-get clean

# install Eigen
WORKDIR /usr/src/
RUN git clone --single-branch --branch 3.3.9 https://gitlab.com/libeigen/eigen eigen3
WORKDIR /usr/src/eigen3/build/
RUN cmake ..
RUN make install -j${N_JOBS}

# Install GTSAM
WORKDIR /usr/src/
RUN git clone --depth 1 --branch 4.0.3 https://github.com/borglab/gtsam.git
WORKDIR /usr/src/gtsam/build
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF \
    ..
RUN make install -j${N_JOBS} && make clean

# add /usr/local/lib to LD_LIBRARY_PATH for dynamic linking
# note: this doesn't seem to save the .bashrc file, which isn't sourced anyway when running the container
#   if not working inside container, run export LD_LIBRARY_PATH=/usr/local/lib:LD_LIBRARY_PATH in the shell
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:LD_LIBRARY_PATH' >> /root/.bashrc

# install HighFive v2.8.0
WORKDIR /usr/src/
RUN git clone --depth 1 --branch v2.8.0 https://github.com/BlueBrain/HighFive
WORKDIR /usr/src/HighFive/build/
RUN cmake ..
RUN make install -j${N_JOBS}
