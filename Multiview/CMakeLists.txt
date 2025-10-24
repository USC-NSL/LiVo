# CMakeList.txt : CMake project for Multiview, include source and define
# project specific logic here.
# Minimum cmake version
cmake_minimum_required (VERSION 3.14 FATAL_ERROR)

# Abseil currently supports C++14, C++17, and C++20.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For Open3D Link:http://www.open3d.org/docs/latest/tutorial/C++/cplusplus_interface.html#ubuntu-macos
if(POLICY CMP0091)
    # https://stackoverflow.com/a/56490614
    cmake_policy(SET CMP0091 NEW)
endif()

# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)

# Add FlatBuffers directly to our build. This defines the `flatbuffers` target.
set(FLATBUFFERS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/flatbuffers)
add_subdirectory(${FLATBUFFERS_SRC_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/flatbuffers-build
                 EXCLUDE_FROM_ALL)

# GFlags
set(GFLAGS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/gflags)
add_subdirectory(${GFLAGS_SRC_DIR})

# ABSEIL
set(ABSEIL_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/abseil-cpp)
add_subdirectory(${ABSEIL_SRC_DIR})

# Find required packages
find_package(PkgConfig REQUIRED)

# Find GStreamer package 
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# DRACO
find_package(draco REQUIRED CONFIG)
if (NOT draco_FOUND)
	message(FATAL_ERROR "Can't find libdraco")
endif()

# Typically you don't care so much for a third party library's tests to be
# run from your own project's code.
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/json)
add_subdirectory(${JSON_SRC_DIR})

# Find external packages
find_package(PCL 1.12.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options filesystem date_time chrono serialization system json)

# Find installed OpenCV
list(APPEND CMAKE_PREFIX_PATH "/home/lei/opencv/opencv-3.4.20_install")
find_package(OpenCV REQUIRED)

# Find ZSTD
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED libzstd)

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

# Open3D
# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
option(BUILD_SHARED_LIBS ON)
option(BUILD_PYTHON_MODULE ON)
option(BUILD_EXAMPLES OFF)
option(BUILD_CUDA_MODULE ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
list(APPEND CMAKE_PREFIX_PATH "/home/lei/open3d_install")
find_package(Open3D REQUIRED)
message("Open3D_INCLUDE_DIRS: ${Open3D_INCLUDE_DIRS}")

message("Project Path: ${PROJECT_SOURCE_DIR}")

# Specify additional directories for the linker
link_directories(${GST_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# FFmpeg doesn't support find_package(). PkgConfig (sudo apt install pkg-config) can be used to link the libraries more conveniently 

# Adding files from KINECT SDK folder - Change this path to Azure Sensor SDK folder in your machine
# Rajrup: To Do: Later add the SDK to the lib folder and point to CMakeLists
set(KINECT_SDK_PATH "/home/lei/Azure-Kinect-Sensor-SDK")
set(KINECT_TOOLS_PATH "/usr/lib/x86_64-linux-gnu/libk4a1.4/")

list(APPEND KINECT_SDK_LIBS 
	${KINECT_SDK_PATH}/build/bin/libk4a.so
	${KINECT_SDK_PATH}/build/bin/libk4arecord.so)

# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# List of libraries to link
LIST(APPEND ABSEIL_LIBS absl::log absl::flat_hash_map)
LIST(APPEND GFLAGS_LIBS gflags::gflags)
LIST(APPEND FLATBUFFERS_LIBS flatbuffers)
LIST(APPEND JSON_LIBS nlohmann_json::nlohmann_json)
LIST(APPEND THREAD_LIBS Threads::Threads OpenMP::OpenMP_CXX rt)
LIST(APPEND DRACO_LIBS -Wl,--whole-archive draco -Wl,--no-whole-archive)
LIST(APPEND OPEN3D_INCLUDES ${Open3D_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS}/open3d ${Open3D_INCLUDE_DIRS}/open3d/3rdparty) 

# Include directories
include_directories(
					${GST_INCLUDE_DIRS}									# GStreamer directories
					${GST_APP_INCLUDE_DIRS}								# GStreamer App directories
					${PCL_INCLUDE_DIRS} 								# PCL directories
					${Boost_INCLUDE_DIRS} 								# Boost libraries
					${PROJECT_SOURCE_DIR}/Multiview/src 				# Source Files
					${PROJECT_SOURCE_DIR}/Multiview/include 			# Header Files
					${PROJECT_SOURCE_DIR}/Multiview/include/Server
					${PROJECT_SOURCE_DIR}/Multiview/include/GeoDist
					${PROJECT_SOURCE_DIR}/Multiview/include/test 		# Header Files for test
					${KINECT_SDK_PATH}/include 							# Kinect SDK Header Files
					${EIGEN3_INCLUDE_DIR}
					${FLATBUFFERS_INCLUDE_DIR}
					${GFLAGS_INCLUDE_DIR}
					${ABSEIL_SRC_DIR}
					${ZSTD_INCLUDE_DIRS}
					${OPEN3D_INCLUDES}
					)

################################ PIPELINES ########################################

# Previous: RGBD based Pipeline Server (with gstreamer and webrtc)
# add_executable(MultiviewServerPool 
# 				"src/server_pool.cpp"
# 				# "src/MultiviewPointCloud.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/shm.cpp"
# 				"src/voxel.cpp"
# 				"src/kalman.cpp"
# 				"src/Panoptic/DataPlayback.cpp"
# 				"src/Server/MultiviewServer.cpp"
# 				"src/Server/MultiviewPointCloudServer.cpp"
# 				"src/Server/WebRTCServer.cpp"
# 				"src/test/colorized_depth.cpp"
# 				"src/Server/MultiviewServerPool.cpp"
# 				"src/mahimahi.cpp"
# 				)

# target_link_libraries(MultiviewServerPool PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS})

add_executable(MultiviewServerPoolNew 
				"src/server_pool_new.cpp"
				# "src/MultiviewPointCloud.cpp"
				"src/PointCloudViewer.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/k4autils.cpp"
				"src/utils.cpp"
				"src/timer.cpp"
				"src/filter.cpp"
				"src/frustum.cpp" 
				"src/octree.cpp"
				"src/shm.cpp"
				"src/voxel.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				"src/Server/MultiviewServer.cpp"
				"src/Server/MultiviewPointCloudServer.cpp"
				# "src/Server/WebRTCServer.cpp"
				"src/Server/WebRTCSender.cpp"
				"src/Server/BitrateSplitterServer.cpp"
				"src/Server/NetworkSender.cpp"
				"src/Server/SenderFrameBuffer.cpp"
				"src/test/colorized_depth.cpp"
				"src/Server/MultiviewServerPoolNew.cpp"
				"src/mahimahi.cpp"
				)

target_link_libraries(MultiviewServerPoolNew PRIVATE ${KINECT_SDK_LIBS} ${GST_LIBRARIES} ${GST_APP_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS})

# Previous: RGBD based Pipeline Client (with gstreamer and webrtc)
# add_executable(MultiviewClientPool 
# 				"src/client_pool.cpp"
# 				# "src/MultiviewPointCloud.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/shm.cpp"
# 				"src/voxel.cpp"
# 				# "src/kalman.cpp"
# 				"src/Panoptic/DataPlayback.cpp"
# 				"src/Client/MultiviewClient.cpp"
# 				"src/Client/MultiviewPointCloudClient.cpp"
# 				"src/Client/PointCloudStreamer.cpp"
# 				"src/Client/WebRTCClient.cpp"
# 				"src/test/colorized_depth.cpp"
# 				"src/Client/MultiviewClientPool.cpp"
# 				"src/mahimahi.cpp"
# 				)	
				
# target_link_libraries(MultiviewClientPool PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

add_executable(MultiviewClientPoolNew 
				"src/client_pool_new.cpp"
				# "src/MultiviewPointCloud.cpp"
				"src/PointCloudViewer.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/k4autils.cpp"
				"src/utils.cpp"
				"src/timer.cpp"
				"src/filter.cpp"
				"src/frustum.cpp" 
				"src/octree.cpp"
				"src/shm.cpp"
				"src/voxel.cpp"
				# "src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				"src/Client/MultiviewClient.cpp"
				"src/Client/MultiviewPointCloudClient.cpp"
				"src/Client/PointCloudStreamer.cpp"
				# "src/Client/WebRTCClient.cpp"
				"src/Client/WebRTCReceiver.cpp"
				"src/Client/BitrateSplitterClient.cpp"
				"src/Client/NetworkReceiver.cpp"
				"src/Client/ReceiverFrameBuffer.cpp"
				"src/test/colorized_depth.cpp"
				"src/Client/MultiviewClientPoolNew.cpp"
				"src/mahimahi.cpp"				
				)	
				
target_link_libraries(MultiviewClientPoolNew PRIVATE ${KINECT_SDK_LIBS} ${GST_LIBRARIES} ${GST_APP_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

# Standalone RGBD based pipeline (without gstreamer)
# add_executable(MultiviewPipeline 
# 				"src/pipeline.cpp"
# 				# "src/MultiviewPointCloud.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				# "src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/voxel.cpp"
# 				"src/kalman.cpp"
# 				"src/Panoptic/DataPlayback.cpp"
# 				"src/Server/MultiviewServer.cpp"
# 				"src/Server/MultiviewServerPoolTest.cpp"
# 				"src/Server/MultiviewPointCloudServer.cpp"
# 				"src/Client/MultiviewClient.cpp"
# 				"src/test/colorized_depth.cpp"
# 				"src/Client/MultiviewPointCloudClient.cpp"
# 				"src/Client/PointCloudStreamer.cpp"
# 				)

# target_link_libraries(MultiviewPipeline PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

# Single threaded RGBD based pipeline (without gstreamer)
# add_executable(PanopticPlaybackServerClient 
# 				"src/Panoptic/panoptic_playback.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				# "src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/voxel.cpp"
# 				"src/kalman.cpp"
# 				"src/Panoptic/DataPlayback.cpp"
# 				"src/Server/MultiviewServer.cpp"
# 				"src/Server/MultiviewPointCloudServer.cpp"
# 				"src/Client/MultiviewClient.cpp"
# 				"src/test/colorized_depth.cpp"
# 				"src/Client/MultiviewPointCloudClient.cpp"
# 				"src/Client/PointCloudStreamer.cpp"
# 				)
				
# target_link_libraries(PanopticPlaybackServerClient PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

# Standalone RGBD based pipeline (without gstreamer) to generate ground truth RGB and Ptcl
add_executable(PanopticGT 
				"src/Panoptic/panoptic_standalone_gt.cpp"
				"src/PointCloudViewer.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/k4autils.cpp"
				"src/utils.cpp"
				"src/timer.cpp"
				# "src/filter.cpp"
				"src/frustum.cpp" 
				"src/octree.cpp"
				"src/voxel.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				"src/Server/MultiviewServer.cpp"
				"src/Server/MultiviewPointCloudServer.cpp"
				"src/Client/MultiviewClient.cpp"
				"src/test/colorized_depth.cpp"
				"src/Client/MultiviewPointCloudClient.cpp"
				"src/Client/PointCloudStreamer.cpp"
				)
				
target_link_libraries(PanopticGT PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

# Standalone Ptcl based pipeline (without TCP) to generate ground truth RGB and Ptcl
add_executable(PanopticPtcl 
				"src/Panoptic/panoptic_standalone_ptcl.cpp"
				"src/PointCloudViewer.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/k4autils.cpp"
				"src/utils.cpp"
				"src/timer.cpp"
				# "src/filter.cpp"
				"src/frustum.cpp" 
				"src/octree.cpp"
				"src/voxel.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				"src/Server/MultiviewServer.cpp"
				"src/Server/MultiviewPointCloudServer.cpp"
				"src/Client/MultiviewClient.cpp"
				"src/test/colorized_depth.cpp"
				"src/Client/MultiviewPointCloudClient.cpp"
				"src/Client/PointCloudStreamer.cpp"
				)
				
target_link_libraries(PanopticPtcl PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

################################## EXPERIMENTS #################################

add_executable(PanotpicPtclTestCompression
				"src/Panoptic/panoptic_ptcl_test_compression.cpp"
				"src/PointCloudViewer.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/k4autils.cpp"
				"src/utils.cpp"
				"src/timer.cpp"
				"src/frustum.cpp" 
				"src/octree.cpp"
				"src/voxel.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				"src/Server/MultiviewServer.cpp"
				"src/Server/MultiviewPointCloudServer.cpp"
				"src/Client/MultiviewClient.cpp"
				"src/test/colorized_depth.cpp"
				"src/Client/MultiviewPointCloudClient.cpp"
				)

target_link_libraries(PanotpicPtclTestCompression PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

add_executable(PanopticPtclDracoBitrateTable
				"src/Panoptic/panoptic_ptcl_draco_bitrate_table.cpp"
				"src/PointCloudViewer.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/k4autils.cpp"
				"src/utils.cpp"
				"src/timer.cpp"
				"src/frustum.cpp" 
				"src/octree.cpp"
				"src/voxel.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				"src/Server/MultiviewServer.cpp"
				"src/Server/MultiviewPointCloudServer.cpp"
				"src/Client/MultiviewClient.cpp"
				"src/test/colorized_depth.cpp"
				"src/Client/MultiviewPointCloudClient.cpp"
				)

target_link_libraries(PanopticPtclDracoBitrateTable PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

add_executable(PanopticPtclDracoOutputGeneration
				"src/Panoptic/panoptic_ptcl_draco_output_generation.cpp"
				"src/PointCloudViewer.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/k4autils.cpp"
				"src/utils.cpp"
				"src/timer.cpp"
				"src/frustum.cpp" 
				"src/octree.cpp"
				"src/voxel.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				"src/Server/MultiviewServer.cpp"
				"src/Server/MultiviewPointCloudServer.cpp"
				"src/Client/MultiviewClient.cpp"
				"src/test/colorized_depth.cpp"
				"src/Client/MultiviewPointCloudClient.cpp"
				)

target_link_libraries(PanopticPtclDracoOutputGeneration PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${DRACO_LIBS} ${THREAD_LIBS} ${Open3D_LIBRARIES})

add_executable(PanopticPrepareData
				"src/Panoptic/prepare_data.cpp"
				"src/calibration.cpp"
				"src/marker.cpp"
				"src/utils.cpp"
				)

target_link_libraries(PanopticPrepareData PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

################################## RANDOM TESTING #################################

add_executable(Open3DVisualize
				"src/Client/Open3DVisualizer.cpp"
				"src/timer.cpp")			

target_link_libraries(Open3DVisualize PRIVATE ${Open3D_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

add_executable(frustum_test
				"src/test/frustum_test.cpp"
				"src/utils.cpp"
				"src/filter.cpp"
				"src/frustum.cpp" 
				"src/voxel.cpp"
				"src/test/utils.cpp"
				)
target_link_libraries(frustum_test PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS})

add_executable(kalman_visualize_test
				"src/test/kalman_visualize_test.cpp"
				"src/utils.cpp"
				"src/filter.cpp"
				"src/frustum.cpp" 
				"src/voxel.cpp"
				"src/timer.cpp"
				"src/test/utils.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				)

target_link_libraries(kalman_visualize_test PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS} ${Open3D_LIBRARIES})

# NEW
add_executable(kalman_test_new
				"src/test/kalman_test_new.cpp"
				"src/utils.cpp"
				"src/filter.cpp"
				"src/frustum.cpp" 
				"src/voxel.cpp"
				"src/timer.cpp"
				"src/test/utils.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				)
target_link_libraries(kalman_test_new PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS})

# OLD
add_executable(kalman_test
				"src/test/kalman_test.cpp"
				"src/utils.cpp"
				"src/filter.cpp"
				"src/frustum.cpp" 
				"src/voxel.cpp"
				"src/timer.cpp"
				"src/test/utils.cpp"
				"src/kalman.cpp"
				"src/Panoptic/DataPlayback.cpp"
				)
target_link_libraries(kalman_test PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS})

# add_executable(test "src/test/test.cpp" "src/octree.cpp" "src/frustum.cpp" "src/timer.cpp")

# target_link_libraries(test PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS}  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS})

add_executable(ffmpeg_test "src/test/ffmpeg.cpp")

target_link_libraries(ffmpeg_test PRIVATE FFmpeg ${KINECT_SDK_LIBS} ${OpenCV_LIBS}  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS})

add_executable(qrcode_test 
				"src/test/qrcode_test.cpp"
				"src/utils.cpp"
				"src/timer.cpp")

target_link_libraries(qrcode_test PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${THREAD_LIBS} ${Open3D_LIBRARIES})

add_executable(draco_encoder_test
				"src/test/draco_encoder_test.cpp"
				"src/utils.cpp"
				"src/timer.cpp")
target_link_libraries(draco_encoder_test PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS} ${THREAD_LIBS} ${DRACO_LIBS})

add_executable(draco_decoder_test
				"src/test/draco_decoder_test.cpp"
				"src/utils.cpp"
				"src/timer.cpp")
target_link_libraries(draco_decoder_test PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS} ${THREAD_LIBS} ${DRACO_LIBS})

add_executable(gpcc_test
				"src/test/gpcc_test.cpp"
				"src/utils.cpp"
				"src/timer.cpp")
target_link_libraries(gpcc_test PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

# add_executable(mmap_server "src/test/mmap_server.cpp" "src/utils.cpp" "src/k4autils.cpp")

# target_link_libraries(mmap_server PRIVATE flatbuffers ${KINECT_SDK_LIBS} ${OpenCV_LIBS}  ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} rt gflags::gflags)

# target_link_libraries(mmap_server PRIVATE flatbuffers ${KINECT_SDK_LIBS} ${OpenCV_LIBS}  ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

# add_executable(colorized_depth_test "src/test/colorized_depth.cpp" "src/utils.cpp" "src/k4autils.cpp")

# target_link_libraries(colorized_depth_test PRIVATE flatbuffers ${KINECT_SDK_LIBS} ${OpenCV_LIBS}  ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS})

# add_executable(open3d_test "src/test/open3d_test.cpp")	

# target_link_libraries(open3d_test PRIVATE ${Open3D_LIBRARIES} ${THREAD_LIBS})

# OLD. Just to check if KF is working
add_executable(kalman_predictor_test 
				"src/test/kalman_predictor_test.cpp" 
				"src/Panoptic/DataPlayback.cpp" 
				"src/utils.cpp"
				"src/filter.cpp"
				"src/kalman.cpp"
				"src/frustum.cpp" 
				"src/voxel.cpp"
				"src/timer.cpp")

target_link_libraries(kalman_predictor_test PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${JSON_LIBS})

add_executable(rate_limit_test 
				"src/test/rate_limit_test.cpp" 
				"src/timer.cpp")
target_link_libraries(rate_limit_test PRIVATE ${Boost_SYSTEM} ${Boost_LIBRARIES})

add_executable(circular_buffer_test 
				"src/test/circular_buffer.cpp" 
				"src/timer.cpp")

target_link_libraries(circular_buffer_test PRIVATE ${Boost_SYSTEM} ${Boost_LIBRARIES} ${THREAD_LIBS})

add_executable(metric_test 
				"src/test/metric_calc_test.cpp" 
				"src/Client/BitrateSplitterClient.cpp"
				"src/utils.cpp"
				"src/timer.cpp")

target_link_libraries(metric_test PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${JSON_LIBS} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${THREAD_LIBS})

####################################### ARCHIVE ##############################################

# # Add source to this project's executable.
# add_executable(Multiview 
# 				"src/Multiview.cpp" 
# 				"src/MultiviewPointCloud.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				)		

# # TODO: Add tests and install targets if needed.
# target_link_libraries(Multiview PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

# # Add source to this project's executable.
# add_executable(PanopticServerClient 
# 				"src/Panoptic/panoptic.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/voxel.cpp"
# 				"src/kalman.cpp"
# 				"src/Panoptic/DataPlayback.cpp"
# 				"src/Server/MultiviewServer.cpp"
# 				"src/Server/MultiviewPointCloudServer.cpp"
# 				"src/Client/MultiviewClient.cpp"
# 				"src/Client/MultiviewPointCloudClient.cpp"
# 				"src/Client/PointCloudStreamer.cpp"
# 				)
				
# # TODO: Add tests and install targets if needed.
# target_link_libraries(PanopticServerClient PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_SYSTEM} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${ZSTD_LIBRARIES} ${THREAD_LIBS})

# # Add source to this project's executable.
# add_executable(MultiviewServerClient 
# 				"src/main.cpp"
# 				# "src/MultiviewPointCloud.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/shm.cpp"
# 				"src/voxel.cpp"
# 				"src/Client/MultiviewClient.cpp"
# 				"src/Client/MultiviewPointCloudClient.cpp"
# 				"src/Server/MultiviewServer.cpp"
# 				"src/Server/MultiviewPointCloudServer.cpp"
# 				"src/test/colorized_depth.cpp"
# 				)	

# # TODO: Add tests and install targets if needed.
# target_link_libraries(MultiviewServerClient PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

# add_executable(MultiviewServer 
# 				"src/server.cpp"
# 				# "src/MultiviewPointCloud.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/shm.cpp"
# 				"src/voxel.cpp"
# 				"src/Server/MultiviewServer.cpp"
# 				"src/Server/MultiviewPointCloudServer.cpp"
# 				"src/Server/WebRTCServer.cpp"
# 				"src/test/colorized_depth.cpp"
# 				)

# target_link_libraries(MultiviewServer PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

# add_executable(MultiviewClient 
# 				"src/client.cpp"
# 				# "src/MultiviewPointCloud.cpp"
# 				"src/PointCloudViewer.cpp"
# 				"src/calibration.cpp"
# 				"src/marker.cpp"
# 				"src/k4autils.cpp"
# 				"src/utils.cpp"
# 				"src/timer.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/octree.cpp"
# 				"src/shm.cpp"
# 				"src/voxel.cpp"
# 				"src/Client/MultiviewClient.cpp"
# 				"src/Client/MultiviewPointCloudClient.cpp"
# 				"src/Client/WebRTCClient.cpp"
# 				"src/test/colorized_depth.cpp"
# 				)	
				
# target_link_libraries(MultiviewClient PRIVATE ${KINECT_SDK_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ABSEIL_LIBS} ${GFLAGS_LIBS} ${FLATBUFFERS_LIBS} ${JSON_LIBS} ${THREAD_LIBS})

# add_executable(Kalman
# 				"src/kalman.cpp"
# 				"src/utils.cpp"
# 				"src/filter.cpp"
# 				"src/frustum.cpp" 
# 				"src/voxel.cpp"
# 				)
# target_link_libraries(Kalman PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} gflags::gflags)

# Print project info
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Boost Libraries: ${Boost_LIBRARIES}")
message(STATUS "GStreamer Libraries: ${GST_LIBRARIES}")