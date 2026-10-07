include_guard(GLOBAL)

get_filename_component(LIVO_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(LIVO_DEPS_ROOT "${LIVO_ROOT_DIR}/Multiview/lib" CACHE PATH
    "LiVo dependency source/install root")
option(LIVO_STRICT_LOCAL_DEPS
       "Require every dependency from Multiview/lib/<name>/install" OFF)

set(_livo_dependency_names
    opencv open3d pcl gstreamer usrsctp azure-kinect eigen boost flatbuffers
    zstd draco abseil-cpp gflags json)

set(LIVO_LOCAL_PREFIXES "")
set(LIVO_LOCAL_PKGCONFIG_DIRS "")
foreach(_name IN LISTS _livo_dependency_names)
    set(_prefix "${LIVO_DEPS_ROOT}/${_name}/install")
    if(EXISTS "${_prefix}")
        list(PREPEND CMAKE_PREFIX_PATH "${_prefix}")
        list(APPEND LIVO_LOCAL_PREFIXES "${_prefix}")
        foreach(_pkgdir lib/pkgconfig lib64/pkgconfig share/pkgconfig)
            if(EXISTS "${_prefix}/${_pkgdir}")
                list(APPEND LIVO_LOCAL_PKGCONFIG_DIRS "${_prefix}/${_pkgdir}")
            endif()
        endforeach()
    elseif(LIVO_STRICT_LOCAL_DEPS)
        message(FATAL_ERROR
            "Missing local dependency '${_name}' at ${_prefix}. "
            "Run install/build-one.sh ${_name} or install/build-all.sh.")
    endif()
endforeach()
list(REMOVE_DUPLICATES CMAKE_PREFIX_PATH)
list(REMOVE_DUPLICATES LIVO_LOCAL_PKGCONFIG_DIRS)

if(LIVO_LOCAL_PKGCONFIG_DIRS)
    list(JOIN LIVO_LOCAL_PKGCONFIG_DIRS ":" _livo_pkgconfig_prefix)
    if(DEFINED ENV{PKG_CONFIG_PATH} AND NOT "$ENV{PKG_CONFIG_PATH}" STREQUAL "")
        set(ENV{PKG_CONFIG_PATH}
            "${_livo_pkgconfig_prefix}:$ENV{PKG_CONFIG_PATH}")
    else()
        set(ENV{PKG_CONFIG_PATH} "${_livo_pkgconfig_prefix}")
    endif()
endif()

function(livo_report_package name version origin)
    if("${origin}" STREQUAL "")
        set(origin "resolved by package configuration")
    endif()
    message(STATUS "LiVo dependency ${name}: version='${version}' origin='${origin}'")
endfunction()

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 3.3.9 REQUIRED NO_MODULE)
find_package(Boost 1.75 REQUIRED COMPONENTS
    program_options filesystem date_time chrono serialization system json
    iostreams regex thread)
find_package(OpenCV 3.4 REQUIRED CONFIG)
if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.0)
    message(FATAL_ERROR
        "LiVo currently requires OpenCV 3.4.x because calibration.cpp uses "
        "legacy OpenCV headers. Run install/build-one.sh opencv.")
endif()
find_package(PCL 1.12 REQUIRED)
find_package(Open3D REQUIRED CONFIG)
if(DEFINED Open3D_VERSION AND Open3D_VERSION VERSION_LESS 0.17.0)
    message(FATAL_ERROR "LiVo requires Open3D 0.17.0 or newer")
endif()
find_package(draco 1.5.7 REQUIRED CONFIG)
find_package(flatbuffers 25.12.19 REQUIRED CONFIG)
set(GFLAGS_USE_TARGET_NAMESPACE TRUE)
find_package(gflags 2.3 REQUIRED CONFIG)
find_package(absl 20250814 REQUIRED CONFIG)
find_package(nlohmann_json 3.12 REQUIRED CONFIG)

pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
pkg_check_modules(GST_CORE REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

set(_k4a_prefix "${LIVO_DEPS_ROOT}/azure-kinect/install")
find_path(K4A_INCLUDE_DIR
    NAMES k4a/k4a.h
    HINTS "${_k4a_prefix}/include")
find_library(K4A_LIBRARY
    NAMES k4a libk4a
    HINTS "${_k4a_prefix}/lib" "${_k4a_prefix}/lib64")
find_library(K4ARECORD_LIBRARY
    NAMES k4arecord libk4arecord
    HINTS "${_k4a_prefix}/lib" "${_k4a_prefix}/lib64")
if(NOT K4A_INCLUDE_DIR OR NOT K4A_LIBRARY OR NOT K4ARECORD_LIBRARY)
    message(FATAL_ERROR
        "Azure Kinect SDK was not found locally or on the system. "
        "Run install/build-one.sh azure-kinect.")
endif()

if(NOT TARGET AzureKinect::k4a)
    add_library(AzureKinect::k4a SHARED IMPORTED)
    set_target_properties(AzureKinect::k4a PROPERTIES
        IMPORTED_LOCATION "${K4A_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${K4A_INCLUDE_DIR}")
endif()
if(NOT TARGET AzureKinect::k4arecord)
    add_library(AzureKinect::k4arecord SHARED IMPORTED)
    set_target_properties(AzureKinect::k4arecord PROPERTIES
        IMPORTED_LOCATION "${K4ARECORD_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${K4A_INCLUDE_DIR}")
endif()
set(KINECT_SDK_LIBS AzureKinect::k4a AzureKinect::k4arecord)
set(KINECT_SDK_PATH "${_k4a_prefix}")

if(TARGET draco::draco)
    set(LIVO_DRACO_TARGET draco::draco)
elseif(TARGET draco)
    set(LIVO_DRACO_TARGET draco)
else()
    message(FATAL_ERROR "The Draco package did not export a usable target")
endif()

set(ABSEIL_LIBS absl::log absl::flat_hash_map)
set(GFLAGS_LIBS gflags::gflags)
set(FLATBUFFERS_LIBS flatbuffers::flatbuffers)
set(JSON_LIBS nlohmann_json::nlohmann_json)
set(THREAD_LIBS Threads::Threads OpenMP::OpenMP_CXX rt)
set(DRACO_LIBS -Wl,--whole-archive ${LIVO_DRACO_TARGET} -Wl,--no-whole-archive)
set(BOOST_SYSTEM Boost::system)
set(BOOST_SERIALIZATION Boost::serialization)
# PCL's package configuration performs its own Boost lookup and overwrites the
# legacy Boost_LIBRARIES variable. Restore LiVo's complete link set afterward.
set(Boost_LIBRARIES
    Boost::program_options
    Boost::filesystem
    Boost::date_time
    Boost::chrono
    Boost::serialization
    Boost::system
    Boost::json
    Boost::iostreams
    Boost::regex
    Boost::thread)

livo_report_package("Eigen3" "${Eigen3_VERSION}" "${Eigen3_DIR}")
livo_report_package("Boost" "${Boost_VERSION}" "${Boost_DIR}")
livo_report_package("OpenCV" "${OpenCV_VERSION}" "${OpenCV_DIR}")
livo_report_package("PCL" "${PCL_VERSION}" "${PCL_DIR}")
livo_report_package("Open3D" "${Open3D_VERSION}" "${Open3D_DIR}")
livo_report_package("GStreamer" "${GST_CORE_VERSION}" "${GST_CORE_PREFIX}")
livo_report_package("Azure Kinect" "1.4.x" "${K4A_LIBRARY}")
livo_report_package("Draco" "1.5.7" "${draco_DIR}")
livo_report_package("FlatBuffers" "25.12.19" "${flatbuffers_DIR}")
livo_report_package("gflags" "2.3.0" "${gflags_DIR}")
livo_report_package("Abseil" "20250814.2" "${absl_DIR}")
livo_report_package("nlohmann_json" "${nlohmann_json_VERSION}" "${nlohmann_json_DIR}")
livo_report_package("zstd" "${ZSTD_VERSION}" "${ZSTD_PREFIX}")
