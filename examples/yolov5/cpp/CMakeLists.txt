cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_EXE_LINKER_FLAGS "-rdynamic")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

# set(CMAKE_CXX_FLAGS_RELEASE "-O2 -fno-rtti -fno-exceptions -DNDEBUG")
# set(CMAKE_EXE_LINKER_FLAGS "-s")

# set(CMAKE_C_FLAGS_RELEASE "-Os -ffunction-sections -fdata-sections")
# set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS_RELEASE "-Os -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -s")

# set(CMAKE_BUILD_TYPE Debug)


set(ULBIC_DIR /home/book/zc/app_sdk_v0.0.2.git/manhattan_doc/prebuilt/arm-rockchip830-linux-uclibcgnueabihf)


set (CMAKE_C_COMPILER ${ULBIC_DIR}/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc)
set (CMAKE_CXX_COMPILER ${ULBIC_DIR}/bin/arm-rockchip830-linux-uclibcgnueabihf-g++)


set(My_SO 1)
if(My_SO EQUAL 1)
    project(rkipc_v1.4_camera_so)
elseif(My_SO EQUAL 0 )  
    project(rkipc_v1.4_camera_app)
endif()


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

message( xxx CMAKE_CURRENT_SOURCE_DIR => ${CMAKE_CURRENT_SOURCE_DIR})

# file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)




if(My_SO EQUAL 1)
    add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    utils_biu.cpp
    ${rknpu_yolov5_file}
    )
elseif(My_SO EQUAL 0 )  
    file(GLOB urat_files hd_uart/src/*.c)
    add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    utils_biu.cpp
    ${rknpu_yolov5_file}
    ${urat_files}
    )
endif()


set(My_Root /home/book/proj/yo/camera-rv1103/examples/yolov5/cpp)

set(OpenCV_INCLUDE_DIRS ${My_Root}/lib_opencv/arm32_rockchip830_install/include/opencv4/)
set(OpenCV_LIBS ${My_Root}/lib_opencv/arm32_rockchip830_install/lib/)

include_directories(${My_Root}/lib_opencv/arm32_rockchip830_install/include/opencv4)

message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV Include Path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${My_Root}/include)
include_directories(${My_Root}/hd_uart/include)

link_directories(${My_Root}/libs)

message(STATUS "Link directories: ${CMAKE_LINK_DIRECTORIES}")

if(My_SO EQUAL 1)
##### so #####
target_link_libraries(${PROJECT_NAME} -Wl,--no-as-needed imageutils fileutils imagedrawing stdc++fs ${LIBRKNNRT}
        ${My_Root}/lib_opencv/arm32_rockchip830_install/lib/libopencv_core.so
        ${My_Root}/lib_opencv/arm32_rockchip830_install/lib/libopencv_imgcodecs.so
        ${My_Root}/lib_opencv/arm32_rockchip830_install/lib/libopencv_imgproc.so
        pthread 
        ${My_Root}/libs/librockit.so 
        ${My_Root}/libs/librockchip_mpp.so
        ${My_Root}/libs/librkaiq.so
        ${My_Root}/libs/libaec_bf_process.so
        ${My_Root}/libs/librkaudio_common.so 
        m 
        ${My_Root}/libs/libfreetype.so
        ${My_Root}/libs/librockiva.so
        ${My_Root}/libs/libiconv.so 
        ${My_Root}/libs/librknnmrt.so 
        ${My_Root}/libs/librga.so
        stdc++ 
        ${My_Root}/libs/librksysutils.so
        ${My_Root}/libs/libapp_shared.so
        
        ${My_Root}/libs/libhd_uart.so
    
        ${My_Root}/lib_libz/arm32_rockchip830_install/lib/libz.so
        dl

    )

elseif(My_SO EQUAL 0 )  
    
##### app ######
      target_link_libraries(${PROJECT_NAME} -Wl,--no-as-needed imageutils fileutils imagedrawing stdc++fs ${LIBRKNNRT}
        ${My_Root}/lib_opencv/arm32_rockchip830_install/lib/libopencv_core.so
        ${My_Root}/lib_opencv/arm32_rockchip830_install/lib/libopencv_imgcodecs.so
        ${My_Root}/lib_opencv/arm32_rockchip830_install/lib/libopencv_imgproc.so
        pthread 
        ${My_Root}/libs/librockit.so 
        ${My_Root}/libs/librockchip_mpp.so
        ${My_Root}/libs/librkaiq.so
        ${My_Root}/libs/libaec_bf_process.so
        ${My_Root}/libs/librkaudio_common.so 
        m 
        ${My_Root}/libs/libfreetype.so
        ${My_Root}/libs/librockiva.so
        ${My_Root}/libs/libiconv.so 
        ${My_Root}/libs/librknnmrt.so 
        ${My_Root}/libs/librga.so
        stdc++ 
        ${My_Root}/libs/librksysutils.so
        ${My_Root}/libs/libapp_shared.so
        ${My_Root}/lib_libz/arm32_rockchip830_install/lib/libz.so
        dl
      )
endif()


add_compile_options(-Wno-psabi)

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
