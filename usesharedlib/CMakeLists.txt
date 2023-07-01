cmake_minimum_required(VERSION 3.12.0)
set (CMAKE_CXX_STANDARD 14)

if(POLICY CMP0091)
    # https://stackoverflow.com/a/56490614
    cmake_policy(SET CMP0091 NEW)
endif()

LINK_DIRECTORIES(
	"/usr/lib/"
)


project(DetectBlocksDemo LANGUAGES C CXX)

#find_library(DetectBlocks_LIB /home/valerie/Open3d_example/sharedlib/build/libDetectBlocks.so)
#message(STATUS ${PRIMEUTIL_LIB})

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/cmake)
find_package(DetectBlocks HINTS /home/valerie/Open3d_example/sharedlib/build)
include_directories(/home/valerie/Open3d_example/sharedlib/include)


add_executable(DetectBlocksDemo src/demo2.cpp ../../sharedlib/src/detectblocks.cpp)

#TARGET_LINK_LIBRARIES(DetectBlocksDemo PUBLIC "detectblocks" Open3D::Open3D )
TARGET_LINK_LIBRARIES(DetectBlocksDemo PUBLIC  Open3D::Open3D )
