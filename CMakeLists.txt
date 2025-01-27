cmake_minimum_required(VERSION 2.8.3)

project(tof_cam)

# Set cpp 14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure OpenCV (for tof_show)
find_package(OpenCV REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  SYSTEM
)

###########
## Build ##
###########

# Set build type and output locations
set(CMAKE_BUILD_TYPE Release)	
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "lib")

# Make shared library
add_library(tof_cam SHARED
  src/tof_cam.cpp
  src/jhcTofCam.cpp
)

# Required input libraries for shared lib
target_link_libraries(tof_cam
  pthread
)


# Make test program to save images
add_executable(tof_save
  src/tof_save.cpp
  src/jhcTofCam.cpp
)

# Required input libraries for saving images
target_link_libraries(tof_save
  pthread
)


# Make test program to show images
add_executable(tof_show
  src/tof_show.cpp
  src/jhcTofCam.cpp
)

# Required input libraries for showing images
target_link_libraries(tof_show
  pthread
  ${OpenCV_LIBS}
)