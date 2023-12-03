<h1 align="center">
  Bioslam: IMU-Based Human Skeletal Pose Estimation<br>
  <a href="https://github.com/tmcg0/bioslam/actions/workflows/build-test-linux.yml/badge.svg"><img src="https://github.com/tmcg0/bioslam/actions/workflows/build-test-linux.yml/badge.svg"/></a> <a href="https://github.com/tmcg0/bioslam/actions/workflows/cpp-linter.yml/badge.svg"><img src="https://github.com/tmcg0/bioslam/actions/workflows/cpp-linter.yml/badge.svg"/></a> <a href="https://codecov.io/gh/tmcg0/bioslam" ><img src="https://codecov.io/gh/tmcg0/bioslam/branch/main/graph/badge.svg?token=7YN205FLRJ"/></a> <a href="https://github.com/tmcg0/bioslam/blob/main/LICENSE.txt"><img src="https://img.shields.io/badge/License-MIT-maroon.svg"/></a>

  <a href="https://github.com/tmcg0/bioslam/actions/workflows/dockerfile-test-v1.0.1.yml/badge.svg"><img src="https://github.com/tmcg0/bioslam/actions/workflows/dockerfile-test-v1.0.1.yml/badge.svg"/></a>
  <a href="https://github.com/tmcg0/bioslam/actions/workflows/dockerfile-test-v1.1.yml/badge.svg"><img src="https://github.com/tmcg0/bioslam/actions/workflows/dockerfile-test-v1.1.yml/badge.svg"/></a>
  <a href="https://github.com/tmcg0/bioslam/actions/workflows/dockerfile-test-v1.2.yml/badge.svg"><img src="https://github.com/tmcg0/bioslam/actions/workflows/dockerfile-test-v1.2.yml/badge.svg"/></a>
</h1>

## What is Bioslam?

Bioslam is a C++/MATLAB toolbox for estimation of human skeletal pose from IMU data, using robust factor-graph based global optimization techniques.

<p align="center">
  <img width="380" height="380" src="https://github.com/tmcg0/bioslam/blob/main/doc/media/bioslam-gait.gif">
  <img width="410" height="410" src="https://github.com/tmcg0/bioslam/blob/main/doc/media/subj_cal_partial.gif">
</p>

## Citing Bioslam

If using this software repository, please cite bioslam through the following Zenodo DOI: 
[![DOI](https://zenodo.org/badge/321773016.svg)](https://zenodo.org/badge/latestdoi/321773016)

### Core publications
<!-- For now I'm putting all of these in IEEE style without ref numbers and without DOI numbers since I'll provide direct links -->
If using bioslam, also please cite the following publication:

- T. McGrath and L. Stirling, “Body-Worn IMU Human Skeletal Pose Estimation Using a Factor Graph-Based Optimization Framework,” *Sensors*, vol. 20, no. 23, p. 6887, Dec. 2020. [[link]](https://doi.org/10.3390/s20236887)

- T. McGrath and L. Stirling, “Body-Worn IMU-Based Human Hip and Knee Kinematics Estimation during Treadmill Walking,” *Sensors*, vol. 22, no. 7, p. 2544, Mar. 2022. [[link]](https://doi.org/10.3390/s22072544)

### Related publications
The novel hinge joint kinematic model is based on:

- T. McGrath, R. Fineman, and L. Stirling, “An Auto-Calibrating Knee Flexion-Extension Axis Estimator Using Principal Component Analysis with Inertial Sensors,” *Sensors*, vol. 18, no. 6, p. 1882, Jun. 2018. [[link]](https://www.mdpi.com/1424-8220/18/6/1882)

The optimization backend ([GTSAM](https://github.com/borglab/gtsam)) and IMU dynamics/preintegration is based on:

- L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert, “Eliminating conditionally independent sets in factor graphs: A unifying perspective based on smart factors,” in *Proceedings - IEEE International Conference on Robotics and Automation*, 2014. [[link]](https://ieeexplore.ieee.org/abstract/document/6907483)

- C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation,” in *Robotics: Science and Systems*, 2015. [[link]](http://www.roboticsproceedings.org/rss11/p06.pdf)
 
## Installation

### Supported systems
Bioslam is tested on the following systems:

- Ubuntu 20.04 with GCC 7, GCC 9, and Clang 9

using Boost 1.67, GTSAM 4.0.3, and Eigen 3.3.9.

On other systems, a bioslam-installed Ubuntu image can be spun up in a Docker container using a provided [Dockerfile](docker/).

### Required Dependencies
#### bioslam < v1.1
* CMake (>= 3.17)
* boost (>= 1.65 && <1.74)
  * Ubuntu: `sudo apt-get install libboost-all-dev`
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 3.3.9+
* hdf5
  * Ubuntu: `sudo apt-get install libhdf5-serial-dev`
* [gtsam 4.0.3](https://github.com/borglab/gtsam) (note: bioslam MATLAB wrapper may have a [known compatibility issue](https://github.com/tmcg0/bioslam/issues/4).)
* [imuDataUtils](https://github.com/tmcg0/imuDataUtils)
* [HighFive](https://github.com/BlueBrain/HighFive) >= 2.3 && < 2.8
* Optional: Intel's Thread Building Blocks (TBB)
  * ubuntu: `sudo apt install libtbb-dev`

#### bioslam >= v1.1 && <1.2
_similar to < v1.1, but:_
* no need for [imuDataUtils](https://github.com/tmcg0/imuDataUtils)

#### bioslam >= 1.2
_similar to v1.1, but:_
* boost >= 1.74
* [gtsam 4.2](https://github.com/borglab/gtsam)
* [HighFive](https://github.com/BlueBrain/HighFive) >= 2.7

### Optional Recommended Dependencies
for faster performance

* [Intel MKL](https://software.intel.com/en-us/mkl) can offer situationally-dependent performane improvements
 * NOTE: MKL is not compatible with Eigen 3.3.4 because of a bug in Eigen. Use a newer version of Eigen.

Note: if any of these three optional dependencies are installed, you must change a corresponding GTSAM flag to use them:
* `GTSAM_WITH_EIGEN_MKL=ON` Tells Eigen to use Intel MKL if available
* `GTSAM_WITH_TBB=ON` Tells GTSAM to use Intel TBB if available
* set `MKL_FFTW_INCLUDE_DIR`, `MKL_INCLUDE_DIR`, `MKL_ROOT_DIR` if you wanna use MKL
* set `TBB_INCLUDE_DIRS`, `TBB_tbb_LIBRARY_RELEASE`, `TBB_tbbmalloc_LIBRARY_RELEASE` if you wanna use TBB
  

### Installation Instructions
#### Installing library via CMake
Starting from the root of the repo:
```sh
mkdir build
cd build
cmake ..
make install # may require root privileges
make test # optional, runs unit tests
```

## Testing
```
make test # remember to `make install` first!
```

Want more verbose output (including `std::cout`) while testing?

```sh
# from build/
CTEST_OUTPUT_ON_FAILURE=1 make test
```

## Development

A convenient [devcontainer configuration](/.devcontainer) is provided for use with VSCode's [Dev Containers extension](https://code.visualstudio.com/docs/devcontainers/containers).

## Bioslam MATLAB interface
In the `matlab/` directory there is an implementation of bioslam for MATLAB. This feature is experimental and prone to instability.

### Building bioslam for MATLAB
Set the appropriate CMake option in bioslam's CMakelists.txt to build the MATLAB wrapper. Then in the `build/` folder (or wherever you build bioslam) there will be a directory called `wrap/`. The contents of this directory will include a directory `wrap/bioslam/+bioslam`. The `+bioslam/` directory must be added to your MATLAB path. Additionally, so does the file `wrap/bioslam_mex/bioslam_wrapper.mexa64`. These will be installed if you `make install`.

#### Required MATLAB Toolboxes
- Bioinformatics Toolbox
- Deep Learning Toolbox
- Robotics System Toolbox
- Mapping Toolbox

### Notes
* Depending on your system and MATLAB configuration, the version of `libstdc++` may be different between MATLAB and your system. This can cause errors. In these cases, it might be easiest to just remove the version of `libstdc++` that shipped with MATLAB, forcing MATLAB to find it on your system instead. Note: this requires it be properly added to your system environment paths (below). [Related question/answer on MATLAB central](https://www.mathworks.com/matlabcentral/answers/364543-why-does-matlab-r2017b-display-erroneous-message-about-libgiolibproxy-so-on-ubuntu-17-10)
    * You can add the standard location for MATLAB with `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib/x86_64-linux-gnu/` in .bashrc
* You will need to set your system environment variables for `LD_RUN_PATH` and `LD_LIBRARY_PATH` for MATLAB to find the built libraries for GTSAM and bioslam. Add the following lines to your .bashrc:
    * `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/`
    * `export LD_RUN_PATH=$LD_RUN_PATH:/usr/local/lib/`
* In general, if `-march=native` is enabled for any libraries, subsequent libraries should pass the same build flag. Make sure that the build option `BIOSLAM_BUILD_WITH_MARCH_NATIVE` matches the GTSAM build option `GTSAM_BUILD_WITH_MARCH_NATIVE`.

## Additional Information

bioslam is open sourced under the MIT license. See [LICENSE](https://github.com/tmcg0/bioslam/blob/dev/LICENSE.txt) for details.
