export GCC_COMPILER=/mnt/c/Users/11198/Documents/RV1103_demo/arm-rockchip830-linux-uclibcgnueabihf-master/bin/arm-rockchip830-linux-uclibcgnueabihf

./build-linux.sh -t rv1103 -a armhf -d yolov5


#ENABLE_ASAN=OFF

add_definitions(-DRV1106_1103)  添加一个宏定义

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/allocator/dma)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../utils/ utils.out)



cmake .. \
  -DTARGET_SOC=rv1106 \
  -DCMAKE_SYSTEM_NAME=Linux \
  -DCMAKE_SYSTEM_PROCESSOR=armhf \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ASAN=OFF \
  -DDISABLE_RGA=OFF \
  -DDISABLE_LIBJPEG=OFF \
  -DCMAKE_INSTALL_PREFIX=/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/install/rv1106_linux_armhf/rknn_yolov5_demo
