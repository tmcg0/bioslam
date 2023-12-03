# --------------------------------------------------------------------------- #
#           bioslam v1.2 Dockerfile for Ubuntu 23.10 base image          
# with dependencies:
#   boost: 1.74.0 (https://packages.ubuntu.com/mantic/libboost-all-dev)
#   HDF5 (serial): 1.10.8 (https://packages.ubuntu.com/mantic/libhdf5-dev)
#   Eigen: 3.3.9 (https://gitlab.com/libeigen/eigen)
#   GTSAM: 4.2 (https://github.com/borglab/gtsam/releases/4.2)
#   HighFive: 2.8.0 (https://github.com/BlueBrain/HighFive/releases/v2.8.0)
# build environment:
#   git: 2.40.1 (https://packages.ubuntu.com/mantic/git)
#   cmake: 3.27.4 (https://packages.ubuntu.com/mantic/cmake)
#   g++: 13.2.0 (from build-essential: https://packages.ubuntu.com/mantic/build-essential)
#   make: 4.3 (from build-essential: https://packages.ubuntu.com/mantic/build-essential)
# notes:
# - only builds the C++ library and executables, no Python/MATLAB wrappers
# - internet connection required to download dependencies
# --------------------------------------------------------------------------- #

FROM ubuntu:23.10

LABEL maintainer="Tim McGrath <t.mike.mcgrath@gmail.com>"

ARG N_JOBS=4

ENV DEBIAN_FRONTEND noninteractive
ENV eigen_version=3.3.9
ENV gtsam_version=4.2
ENV highfive_version=v2.8.0
ENV bioslam_version=v1.2

# install package dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    build-essential \
    git \
    cmake \
    libhdf5-dev \
    libboost-all-dev \
    && apt-get clean

# install Eigen
WORKDIR /usr/src/
RUN git clone --single-branch --branch ${eigen_version} https://gitlab.com/libeigen/eigen.git eigen3
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
RUN cmake \
    -DBIOSLAM_BUILD_MATLAB_WRAPPER=OFF \
    -DBIOSLAM_BUILD_WITH_MARCH_NATIVE=ON \
    -DBIOSLAM_USE_TBB=OFF \
    ..
RUN make install -j${N_JOBS}

ENTRYPOINT ["/bin/bash"]
