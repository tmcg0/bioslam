# Builds a docker image on Ubuntu 20.04 with bioslam, GTSAM 4.0.3, Eigen 3.3.9, and TBB
# Note: only builds C++ libraries, no Python/MATLAB wrappers

# base Ubuntu 20.04
FROM ubuntu:20.04

# add maintainer info
LABEL maintainer="Tim McGrath <t.mike.mcgrath@gmail.com>"

# Disable GUI prompts
ENV DEBIAN_FRONTEND noninteractive

# Update apps on the base image
RUN apt-get -y update && apt-get -y install

# install git
RUN apt-get -y install git-all

# Install C++
RUN apt-get -y install build-essential apt-utils

# Install boost and cmake
RUN apt-get -y install libboost-all-dev cmake

# install Eigen
# RUN apt-get -y install libeigen3-dev
WORKDIR /usr/src/
RUN git clone --single-branch --branch 3.3.9 https://gitlab.com/libeigen/eigen eigen3
WORKDIR /usr/src/eigen3/build/
RUN cmake ..
RUN make install -j4

# Install TBB
RUN apt-get -y install libtbb-dev

# Clone GTSAM 4.0.3
WORKDIR /usr/src/
RUN git clone --depth 1 --branch 4.0.3 https://github.com/borglab/gtsam.git

# Change to build directory. Will be created automatically.
WORKDIR /usr/src/gtsam/build

# Run cmake
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    ..

# Build gtsam
RUN make -j4 install && make clean

# Needed to link with GTSAM
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:LD_LIBRARY_PATH' >> /root/.bashrc

# install hdf5
RUN apt-get -y install libhdf5-serial-dev

# install highfive libraries
WORKDIR /usr/src/
RUN git clone --depth 1 --branch master https://github.com/BlueBrain/HighFive
WORKDIR /usr/src/HighFive/build/
RUN cmake ..
RUN make install -j4

# install imuDataUtils
WORKDIR /usr/src/
RUN git clone --depth 1 --branch master https://github.com/tmcg0/imuDataUtils
WORKDIR /usr/src/imuDataUtils/build/
RUN cmake ..
RUN make install -j4

# install bioslam
WORKDIR /usr/src/
RUN git clone --depth 1 --branch dev https://github.com/tmcg0/bioslam
WORKDIR /usr/src/bioslam/build/
RUN cmake -DBIOSLAM_BUILD_MATLAB_WRAPPER=OFF ..
RUN make install -j4

# Run bash
CMD ["bash"]
