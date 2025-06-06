cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ULBIC_DIR /mnt/c/Users/11198/Documents/C++_libs/arm-rockchip830-linux-uclibcgnueabihf-master)

set (CMAKE_C_COMPILER ${ULBIC_DIR}/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc)
set (CMAKE_CXX_COMPILER ${ULBIC_DIR}/bin/arm-rockchip830-linux-uclibcgnueabihf-g++)

project(ipc_app_0606_camera2)

if (ENABLE_ASAN)
	message(STATUS "BUILD WITH ADDRESS SANITIZER")
	set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
	set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
	set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif ()

set(rknpu_yolov5_file rknpu2/yolov5.cc)
if (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
    add_definitions(-DRV1106_1103)
    set(rknpu_yolov5_file rknpu2/yolov5_rv1106_1103.cc)
    #dma
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/allocator/dma)
elseif(TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    add_definitions(-DRKNPU1)
    set(rknpu_yolov5_file rknpu1/yolov5.cc)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../utils/ utils.out)

#set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")

# file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)


file(GLOB urat_files hd_uart/src/*.c)

add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    utils_biu.cpp
    ${rknpu_yolov5_file}
    # ${urat_files}
)

set(OpenCV_INCLUDE_DIRS /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_install/include/opencv4/)
set(OpenCV_LIBS /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_install/lib/)

include_directories(/mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_install/include/opencv4)

message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV Include Path: ${OpenCV_INCLUDE_DIRS}")

include_directories(/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/include)
include_directories(/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/hd_uart/include)

link_directories(/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs)

message(STATUS "Link directories: ${CMAKE_LINK_DIRECTORIES}")



#target_link_libraries(${PROJECT_NAME}
#    imageutils
#    fileutils
#    imagedrawing    
#    stdc++fs
#    ${LIBRKNNRT}
#    
#    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_without_ffmpeg_install/lib/libopencv_core.so
#    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_without_ffmpeg_install/lib/libopencv_imgcodecs.so
#    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_without_ffmpeg_install/lib/libopencv_imgproc.so
#    
#    pthread 
#    rockit 
#    rockchip_mpp 
#    rkaiq 
#    aec_bf_process 
#    rkaudio_common 
#    m 
#    freetype 
#    rockiva 
#    iconv 
#    rknnmrt 
#    rga 
#    stdc++ 
#    rksysutils 
#    app_shared
#    
#    /mnt/c/Users/11198/Documents/C++_libs/zlib-1.2.11/arm32_rockchip830_install/lib/libz.so
#    dl
#)



target_link_libraries(${PROJECT_NAME} -Wl,--no-as-needed imageutils fileutils imagedrawing stdc++fs ${LIBRKNNRT}
    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_without_ffmpeg_install/lib/libopencv_core.so
    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_without_ffmpeg_install/lib/libopencv_imgcodecs.so
    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_without_ffmpeg_install/lib/libopencv_imgproc.so
    pthread 
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librockit.so 
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librockchip_mpp.so
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librkaiq.so
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/libaec_bf_process.so
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librkaudio_common.so 
    m 
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/libfreetype.so
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librockiva.so
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/libiconv.so 
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librknnmrt.so 
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librga.so
    stdc++ 
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/librksysutils.so
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/libapp_shared.so
    /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs/libhd_uart.so
  
    /mnt/c/Users/11198/Documents/C++_libs/zlib-1.2.11/arm32_rockchip830_install/lib/libz.so
    dl
    
)

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME}
    log
)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} Threads::Threads)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBRKNNRT_INCLUDES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION .)
#install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/bus.jpg DESTINATION ./model)
#install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/coco_80_labels_list.txt DESTINATION ./model)
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model)