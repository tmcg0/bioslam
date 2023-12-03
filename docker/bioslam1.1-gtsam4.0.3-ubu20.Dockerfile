# --------------------------------------------------------------------------- #
#           bioslam v1.1 Dockerfile for Ubuntu 20.04 base image          
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
# - note: prior to bioslam v1.1, imuDataUtils is required
# --------------------------------------------------------------------------- #

FROM ubuntu:20.04

LABEL maintainer="Tim McGrath <t.mike.mcgrath@gmail.com>"

ARG N_JOBS=4

ENV DEBIAN_FRONTEND noninteractive
ENV eigen_version=3.3.9
ENV gtsam_version=4.0.3
ENV highfive_version=v2.8.0
ENV bioslam_version=v1.1

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
RUN git clone --single-branch --branch ${eigen_version} https://gitlab.com/libeigen/eigen eigen3
WORKDIR /usr/src/eigen3/build/
RUN cmake ..
RUN make install -j${N_JOBS}

# Install GTSAM
WORKDIR /usr/src/
RUN git clone --depth 1 --branch ${gtsam_version} https://github.com/borglab/gtsam.git
WORKDIR /usr/src/gtsam/build
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=ON \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_INSTALL_CPPUNITLITE=OFF \
    ..
RUN make install -j${N_JOBS} && make clean


# add /usr/local/lib to LD_LIBRARY_PATH for dynamic linking (in bash only!)
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> /root/.bashrc

# install HighFive
WORKDIR /usr/src/
RUN git clone --depth 1 --branch ${highfive_version} https://github.com/BlueBrain/HighFive
WORKDIR /usr/src/HighFive/build/
RUN cmake ..
RUN make install -j${N_JOBS}

# install bioslam
WORKDIR /usr/src/
RUN git clone --depth 1 --branch ${bioslam_version} https://github.com/tmcg0/bioslam
WORKDIR /usr/src/bioslam/build/
RUN cmake -DBIOSLAM_BUILD_MATLAB_WRAPPER=OFF ..
RUN make install -j${N_JOBS}

ENTRYPOINT ["/bin/bash"]
