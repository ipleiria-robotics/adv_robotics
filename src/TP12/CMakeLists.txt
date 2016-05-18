CMAKE_MINIMUM_REQUIRED(VERSION 2.4 FATAL_ERROR)
PROJECT(TP12)

# Add OpenCV
FIND_PACKAGE( OpenCV REQUIRED )

# Add definitions, compiler switches, etc.
SET(CMAKE_BUILD_TYPE Debug) # Debug
ADD_DEFINITIONS(-O0) # Debug
# ADD_DEFINITIONS(-O2) # Release

# Show all warnings when compiling
ADD_DEFINITIONS(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Add executable
ADD_EXECUTABLE( tp12 src/tp12.cpp )
TARGET_LINK_LIBRARIES(tp12 ${OpenCV_LIBS})
