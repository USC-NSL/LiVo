cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(webrtc_server VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find required packages
find_package(PkgConfig REQUIRED)

# Find GStreamer package 
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# GFlags
set(GFLAGS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/gflags)
add_subdirectory(${GFLAGS_SRC_DIR} gflags)

# ABSEIL
set(ABSEIL_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/abseil-cpp)
set(ABSL_PROPAGATE_CXX_STD ON)
add_subdirectory(${ABSEIL_SRC_DIR} abseil-cpp)

# Find Boost package
find_package(Boost REQUIRED COMPONENTS program_options filesystem date_time chrono serialization system json)

# Find installed OpenCV
list(APPEND CMAKE_PREFIX_PATH "/home/lei/opencv/opencv-3.4.20_install")
find_package(OpenCV REQUIRED)

# Find OpenMP package
find_package(OpenMP)

# We need boost libraries
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
	message(FATAL_ERROR "Can't find libboost_serialization")
endif()

find_library(BOOST_SYSTEM boost_system)
if (NOT BOOST_SYSTEM)
	message(FATAL_ERROR "Can't find libboost_system")
endif()


if(NOT OpenCV_FOUND)
	message(FATAL_ERROR "Can't find OpenCV")
endif()

if(NOT OpenMP_CXX_FOUND)
	message(FATAL_ERROR "Can't find OpenMP")
endif()

# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# List of libraries to link
LIST(APPEND ABSEIL_LIBS absl::log absl::flat_hash_map)
LIST(APPEND GFLAGS_LIBS gflags::gflags)
LIST(APPEND THREAD_LIBS Threads::Threads OpenMP::OpenMP_CXX rt)

include_directories(${GST_INCLUDE_DIRS}
                    ${GST_APP_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
                    "${CMAKE_CURRENT_SOURCE_DIR}/../../../include/"
                    )

# Specify additional directories for the linker
link_directories(${GST_LIBRARY_DIRS})

##################################### TESTING #####################################

# Sender Only C++ Test
add_executable(sender_only 
                sender_only.cpp)

target_link_libraries(sender_only ${GST_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${THREAD_LIBS})

# Sender Only C++ Test
add_executable(sender_webrtc_only 
                sender_webrtc_only.cpp
                "${CMAKE_CURRENT_SOURCE_DIR}/../../timer.cpp"
                WebRTCServer.cpp
                )

target_link_libraries(sender_webrtc_only ${GST_LIBRARIES} ${GST_APP_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${THREAD_LIBS})

# Receiver Only C++ Test
add_executable(receiver_only 
                receiver_only.cpp)

target_link_libraries(receiver_only ${GST_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${THREAD_LIBS})

# Receiver Only C++ Test
add_executable(receiver_webrtc_only 
                receiver_webrtc_only.cpp
                "${CMAKE_CURRENT_SOURCE_DIR}/../../timer.cpp"
                WebRTCReceiver.cpp
                )

target_link_libraries(receiver_webrtc_only ${GST_LIBRARIES} ${GST_APP_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${THREAD_LIBS})

# Sample test from original repo
add_executable(webrtc_server main.cpp)

target_link_libraries(webrtc_server ${GST_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${THREAD_LIBS})

# tiling encoder
add_executable(tiling_encoder tiling_nontiling_encoder.cpp
                "${CMAKE_CURRENT_SOURCE_DIR}/../../timer.cpp"
                TilingEncoderGPU.cpp
                TilingEncoderCPU.cpp
                TilingEncoderCPUParallel.cpp)

target_link_libraries(tiling_encoder ${GST_LIBRARIES} ${GST_APP_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${THREAD_LIBS})

# Set properties
set_target_properties(webrtc_server PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# Print project info
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Boost Libraries: ${Boost_LIBRARIES}")
message(STATUS "GStreamer Libraries: ${GST_LIBRARIES}")
