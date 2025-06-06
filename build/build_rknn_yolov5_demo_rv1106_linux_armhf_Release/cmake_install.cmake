# Install script for directory: /mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/install/rv1106_linux_armhf/rknn_yolov5_demo")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/mnt/c/Users/11198/Documents/C++_libs/arm-rockchip830-linux-uclibcgnueabihf-master/bin/arm-rockchip830-linux-uclibcgnueabihf-objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./ipc_app_0606_camera2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./ipc_app_0606_camera2")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./ipc_app_0606_camera2"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE EXECUTABLE FILES "/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/build/build_rknn_yolov5_demo_rv1106_linux_armhf_Release/ipc_app_0606_camera2")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./ipc_app_0606_camera2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./ipc_app_0606_camera2")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./ipc_app_0606_camera2"
         OLD_RPATH "/mnt/c/Users/11198/Documents/C++_libs/zlib-1.2.11/arm32_rockchip830_install/lib:/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/libs:/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/3rdparty/rknpu2/Linux/armhf-uclibc:/mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm32_rockchip830_without_ffmpeg_install/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/mnt/c/Users/11198/Documents/C++_libs/arm-rockchip830-linux-uclibcgnueabihf-master/bin/arm-rockchip830-linux-uclibcgnueabihf-strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./ipc_app_0606_camera2")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/model" TYPE FILE FILES "/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/../model/one_category_full.rknn")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/build/build_rknn_yolov5_demo_rv1106_linux_armhf_Release/3rdparty.out/cmake_install.cmake")
  include("/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/build/build_rknn_yolov5_demo_rv1106_linux_armhf_Release/utils.out/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/build/build_rknn_yolov5_demo_rv1106_linux_armhf_Release/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
