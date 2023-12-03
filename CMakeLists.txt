cmake_minimum_required(VERSION 3.5)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Set the version number for the library
set (BIOSLAM_VERSION_MAJOR 1)
set (BIOSLAM_VERSION_MINOR 2)
set (BIOSLAM_VERSION_PATCH 0)
math (EXPR BIOSLAM_VERSION_NUMERIC "10000 * ${BIOSLAM_VERSION_MAJOR} + 100 * ${BIOSLAM_VERSION_MINOR} + ${BIOSLAM_VERSION_PATCH}")
set (BIOSLAM_VERSION_STRING "${BIOSLAM_VERSION_MAJOR}.${BIOSLAM_VERSION_MINOR}.${BIOSLAM_VERSION_PATCH}")

project(bioslam VERSION ${BIOSLAM_VERSION_STRING} LANGUAGES CXX C) # added C because some HDF5 tests were .c

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)

# --- guard against in-source builds --- #
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()
# -------------------------------------- #

set(PROJECT_DESCRIPTION "IMU + biomechanics estimation toolbox") # for compatibility with CMake 3.8.2

### ----- User options ----- ###
option(BIOSLAM_BUILD_WITH_MARCH_NATIVE "Build with -march=native (this should match the gtsam build option GTSAM_BUILD_WITH_MARCH_NATIVE)" ON)
option(BIOSLAM_BUILD_TESTS "Should I build these tests?" ON)
option(GTSAM_USE_MKL "Does GTSAM require MKL?" OFF)
option(BIOSLAM_BUILD_MATLAB_WRAPPER "Build the MATLAB interface for bioslam?" OFF)
option(BIOSLAM_BUILD_EXAMPLES "Build examples?" ON)
option(BIOSLAM_USE_TBB "Include Intel's TBB library?" ON) # if GTSAM uses TBB, you need to set this to 'ON'
### ------------------------ ###

# ++++++++++++++++++ handle CMake flags ++++++++++++++++++++ #
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release) # by default build as 'Release'
endif()
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall")
if(BIOSLAM_BUILD_WITH_MARCH_NATIVE)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
  if(COMPILER_SUPPORTS_MARCH_NATIVE) # add -march=native flag, if the compiler supports it
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
  else() # don't add flag, tell user compiler doesn't support it
    message(STATUS "You requested build with -march=native, but detected compiler does not support it. Leaving this flag out.")
  endif()
else() # clear -march=native (in case it was passed by compiler defaults)
  message(STATUS "TODO: remove flags in cmake. current cmake_cxx_flags = ${CMAKE_CXX_FLAGS}. let's hope -march=native isn't in this list.")
endif()
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ #

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/") # include the modules subfolder

include(CodeCoverage)
append_coverage_compiler_flags()

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR cmake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/bioslam)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files  ")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ #
# ++++++++++++++++++ find and use all dependent libraries of bioslam +++++++++++++++++++++ #
set(BIOSLAMLIBS) # variables list for adding libraries to
# --- Require Eigen ---
find_package(Eigen3 REQUIRED)
message(STATUS "found Eigen at: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})
# --- include MKL headers --- # (optional, only required by GTSAM)
if(GTSAM_USE_MKL)
  find_package(MKL)
  include_directories(${MKL_INCLUDE_DIR})
  list(APPEND BIOSLAMLIBS ${MKL_LIBRARIES})
  message(STATUS "including MKL from ${MKL_INCLUDE_DIRS}")
endif()
# --- Require Boost ---
find_package(Boost 1.65 COMPONENTS filesystem REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  list(APPEND BIOSLAMLIBS  ${Boost_LIBRARIES})
  message("found Boost at Boost_INCLUDE_DIR=${Boost_INCLUDE_DIR}")
elseif(NOT Boost_FOUND)
  message("ERROR: Boost not found")
endif()
# --- optionally include TBB if GTSAM wants it ---
if(BIOSLAM_USE_TBB)
  find_package(TBB)
  message(STATUS "setting tbb include dirs to: ${TBB_INCLUDE_DIRS}")
  include_directories(${TBB_INCLUDE_DIRS})
  message(STATUS "tbb libraries are:  ${TBB_LIBRARIES}}")
  list(APPEND BIOSLAMLIBS ${TBB_LIBRARIES})
endif()
# --- Require HDF5 ---
find_package(HDF5 COMPONENTS CXX REQUIRED) # have to find CXX component first for some reason
add_definitions(${HDF5_CXX_DEFINITIONS})
include_directories(${HDF5_CXX_INCLUDE_DIR})
list(APPEND BIOSLAMLIBS ${HDF5_CXX_LIBRARIES})
find_package(HDF5 REQUIRED)
add_definitions(${HDF5_DEFINITIONS})
include_directories(${HDF5_INCLUDE_DIR})
list(APPEND BIOSLAMLIBS ${HDF5_LIBRARIES})
# --- required HighFive --- #
find_package(HighFive REQUIRED)
# --- Require GTSAM ---
find_package(GTSAM REQUIRED)
if (GTSAM_FOUND)
  include_directories(${GTSAM_INCLUDE_DIR})
elseif(GTSAM_NOT_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(GTSAM_FOUND)
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ #
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ #

# make all includes and sources
message(STATUS "proj source dir is ${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include") # bioslam headers
include_directories("${PROJECT_SOURCE_DIR}/include/factors") # bioslam headers for factors
include_directories("${PROJECT_SOURCE_DIR}/include/imu") # bioslam headers for imu
# --- add sources --- #
file(GLOB SOURCES_BIOSLAM "${PROJECT_SOURCE_DIR}/include/*.h" "${PROJECT_SOURCE_DIR}/include/factors/*.h" "${PROJECT_SOURCE_DIR}/include/imu/*.h")
# base stuff
list(APPEND SOURCES_BIOSLAM ${PROJECT_SOURCE_DIR}/src/lowerBodyPoseEstimator.cpp ${PROJECT_SOURCE_DIR}/src/imuPoseEstimator.cpp ${PROJECT_SOURCE_DIR}/src/imu/imuNoiseModelHandler.cpp ${PROJECT_SOURCE_DIR}/src/imu/imu.cpp ${PROJECT_SOURCE_DIR}/src/VarStrToCharMap.cpp)
# add utilities
list(APPEND SOURCES_BIOSLAM ${PROJECT_SOURCE_DIR}/src/gtsamutils.cpp ${PROJECT_SOURCE_DIR}/src/mathutils.cpp ${PROJECT_SOURCE_DIR}/src/testutils.cpp ${PROJECT_SOURCE_DIR}/src/bioutils.cpp)
# add custom factors
list(APPEND SOURCES_BIOSLAM ${PROJECT_SOURCE_DIR}/src/factors/AngleBetweenAxisAndSegmentFactor.cpp ${PROJECT_SOURCE_DIR}/src/factors/AngularVelocityFactor.cpp ${PROJECT_SOURCE_DIR}/src/factors/ConstrainedJointCenterVelocityFactor.cpp ${PROJECT_SOURCE_DIR}/src/factors/HingeJointFactors.cpp ${PROJECT_SOURCE_DIR}/src/factors/ConstrainedJointCenterPositionFactor.cpp)
list(APPEND SOURCES_BIOSLAM ${PROJECT_SOURCE_DIR}/src/factors/MagPose3Factor.cpp ${PROJECT_SOURCE_DIR}/src/factors/Point3Priors.cpp ${PROJECT_SOURCE_DIR}/src/factors/Point3Priors.cpp ${PROJECT_SOURCE_DIR}/src/factors/SegmentLengthDiscrepancyFactor.cpp ${PROJECT_SOURCE_DIR}/src/factors/SegmentLengthMagnitudeFactor.cpp ${PROJECT_SOURCE_DIR}/src/factors/Pose3Priors.cpp)
# ------------------- #
# create and link library
message(STATUS "Libs that we have for bioslam are ${BIOSLAMLIBS} ")
message("bioslam sources are ${SOURCES_BIOSLAM}")
add_library(bioslam SHARED ${SOURCES_BIOSLAM})
set_target_properties(bioslam PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)
target_link_libraries(bioslam ${BIOSLAMLIBS})
target_link_libraries(bioslam gtsam)
target_link_libraries(bioslam HighFive)

if (BIOSLAM_BUILD_MATLAB_WRAPPER) # build the wrapper for MATLAB. this is primarily used to wrap custom bioslam factors.
  message("building bioslam MATLAB wrapper...")
  find_package(GTSAMCMakeTools) # this should find gtsamMatlabWrap.cmake
  include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation (through included wrap_and_install_library function)
  set(BIOSLAM_WRAPPER_LIB "bioslamMatlabWrapper")
  add_library(${BIOSLAM_WRAPPER_LIB} SHARED wrap/bioslam.h)
  set_target_properties(${BIOSLAM_WRAPPER_LIB} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(${BIOSLAM_WRAPPER_LIB} bioslam)
  # Install library
  install(TARGETS ${BIOSLAM_WRAPPER_LIB} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
  # Build MATLAB wrapper (CMake tracks the dependency to link with GTSAM through our project's static library)
  wrap_and_install_library("wrap/bioslam.h" "${BIOSLAM_WRAPPER_LIB}" "" "")
endif()

if(BUILD_MATLAB_WRAPPER AND GTSAM_USE_MKL)
  message(WARNING "There is a known issue with building the MKL version of GTSAM for use with MATLAB. If you want to use MKL, you won't be able to use the MATLAB wrapper.")
endif()

if(BIOSLAM_BUILD_EXAMPLES) # build examples in subdirectory examples/
  message(STATUS "building examples...")
  add_subdirectory(examples)
endif()

# make tests for CTest environment
if(BIOSLAM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

# ========= handle installing and exporting ======== #
file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/include")
file(RELATIVE_PATH CONF_REL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/lib")

# Add all targets to the build-tree export set
export(TARGETS bioslam FILE "${PROJECT_BINARY_DIR}/BioslamTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE bioslam)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/BioslamConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
)

# Create BioslamConfig.cmake with extra info from BioslamConfig.cmake.in
# This file is necessary to find_package the library bioslam.
set(INSTALL_CONFIGDIR lib/cmake/bioslam)
configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/BioslamConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/BioslamConfig.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# install the library
install(TARGETS bioslam EXPORT bioslam-export LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# install the header files
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
        DESTINATION include/bioslam
        FILES_MATCHING PATTERN "*.h")

# Install the BioslamConfig.cmake, BioslamConfigVersion.cmake, and BioslamTargets.cmake files
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/BioslamConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/BioslamConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}")

# install the exported BioslamTargets.cmake
install(EXPORT bioslam-export
        FILE
        BioslamTargets.cmake
        DESTINATION ${INSTALL_CMAKE_DIR}
        )

################################################################################
# Print configuration
message(STATUS "======================================================================")
message(STATUS "==================  Bioslam v${BIOSLAM_VERSION_STRING} Library Options  ==================")
message(STATUS "Build tests?                              : ${BIOSLAM_BUILD_TESTS}")
message(STATUS "Build examples?                           : ${BIOSLAM_BUILD_EXAMPLES}")
message(STATUS "Build MATLAB wrapper?                     : ${BIOSLAM_BUILD_MATLAB_WRAPPER}")
message(STATUS "Use TBB?                                  : ${BIOSLAM_USE_TBB}")
message(STATUS "======================  Configuration Options  =======================")
message(STATUS "System                                    : ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processer architecture                    : ${CMAKE_HOST_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_CXX_COMPILER_ID type                : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "CMake version                             : ${CMAKE_VERSION}")
message(STATUS "CMake generator                           : ${CMAKE_GENERATOR}")
message(STATUS "CMake build tool                          : ${CMAKE_BUILD_TOOL}")
message(STATUS "Build flags                                               ")
if(NOT MSVC AND NOT XCODE_VERSION)
  message(STATUS "  Build type                              : ${CMAKE_BUILD_TYPE}")
  message(STATUS "  C compilation flags                     : ${CMAKE_C_FLAGS}")
  message(STATUS "  C++ compilation flags                   : ${CMAKE_CXX_FLAGS}")
  if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    message(STATUS "  C compilation flags (Release)           : ${CMAKE_C_FLAGS_RELEASE}")
    message(STATUS "  C++ compilation flags (Release)         : ${CMAKE_CXX_FLAGS_RELEASE}")
  endif()
  if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(STATUS "  C compilation flags (Debug)             : ${CMAKE_C_FLAGS_DEBUG}")
    message(STATUS "  C++ compilation flags (Debug)           : ${CMAKE_CXX_FLAGS_DEBUG}")
  endif()
endif()
message(STATUS "======================================================================")
