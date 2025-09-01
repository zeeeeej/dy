// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "yolov5.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"

#include <filesystem>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "utils_biu.hpp"
// #include "hd_uart_parser.h"
#include <fstream>



#include "hd_uart_parser.h"
// #include "hd_camera_protocol.h"

#include <getopt.h>          
#include <linux/input.h>     

  
#include "param.h"
#include "common.h"



#include "rk_mpi_sys.h"
extern "C"{
#include "isp.h"
}

#include "log.h"
#include "network.h"

#include "rockiva.h"
#include "storage.h"
#include "system.h"
#include "photo.h"
#include "uart.h"
#include "data.h"
#include "mpu6887p.h"
#include "heat.h"

#include <memory>
#include <cstdio>

#include <ctime>
#include <chrono>


#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "rkipc.c"


#if defined(RV1106_1103) 
    #include "dma_alloc.hpp"
#endif


// enum { LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG };

// int enable_minilog = 0;
// int rkipc_log_level = LOG_INFO;

#define SCALE_PATH          "/userdata/hadlinks_scale"
#define app_version         "d0.0.7"
#define HD_CAMERA_ADDR      2

static int g_main_run_ = 1;
static char *rkipc_ini_path_ = NULL;
static char *rkipc_iq_file_path_ = NULL;

/*-------------------------------------------
                动态摄像头
-------------------------------------------*/
static rknn_app_context_t rknn_app_ctx;
static int scale_index = 0;
static char scale_path[1024];
using namespace cv;
using namespace std;

namespace mydata {
    std::string action_id_txt_name = "/userdata/action_id.txt";
}

void action_id_collect(uint8_t status, const char *action_id){
   
}

void *on_event(int event_id, void *event_value, size_t event_value_size) {
    return NULL;
}

/*-------------------------------------------
                变换图片
-------------------------------------------*/
/**
 * 将一张图片进行目标检测，返回目标图片。
 * 
 * 步骤：
 * 1.缩放生成小图
 * 2.目标检测生成结果
 * 3.根据结果截图，存入dst_path
 * 
 * @param src_path 原始图片
 * @param dst_path 生成的目标图片
 * @result 0:成功 其他：错误码。
 */
static int process_image_with_yolov5_v2(const std::string& src_path,const std::string& scale_path, int box[5][4], rknn_app_context_t& rknn_app_ctx) {
    int ret = 0;
    std::cout<<"<$>process_image_with_yolov5_v2" << src_path << scale_path <<std::endl;
    resize_images_single(src_path,scale_path, 960);
    printf("process_image_with_yolov5_v2 resize_images_in_folder ok.\n");
    std::vector<std::string> frames;
    frames.push_back(scale_path);
    // std::vector<std::string> frames = get_image_paths(src_path);
    printf("process_image_with_yolov5_v2 get_image_paths ok.\n");
    for (const std::string& img_path : frames) {
        std::cout<< "<1>process_image_with_yolov5_v2 ==> "<<img_path<< std::endl;
        cv::Mat image_change = cv::imread(img_path);
        if (image_change.empty()) {
            std::cerr << "读取图片失败!" << std::endl;
            return -1;
        }
        std::cout<< "<2>process_image_with_yolov5_v2 imread ok "<< std::endl;
        image_buffer_t src_image;
            
        memset(&src_image, 0, sizeof(image_buffer_t));
        ret = read_image(img_path.c_str(), &src_image);
        std::cout<< "<3>process_image_with_yolov5_v2 read_image ok "<< std::endl;

        //RV1106 rga requires that input and output bufs are memory allocated by dma
        ret = dma_buf_alloc(RV1106_CMA_HEAP_PATH, src_image.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                        (void **) & (rknn_app_ctx.img_dma_buf.dma_buf_virt_addr));
        std::cout<< "<4>process_image_with_yolov5_v2 dma_buf_alloc ok "<< std::endl;
        memcpy(rknn_app_ctx.img_dma_buf.dma_buf_virt_addr, src_image.virt_addr, src_image.size);
        dma_sync_cpu_to_device(rknn_app_ctx.img_dma_buf.dma_buf_fd);
        std::cout<< "<5>process_image_with_yolov5_v2 dma_sync_cpu_to_device ok "<< std::endl;
        free(src_image.virt_addr);
        src_image.virt_addr = (unsigned char *)rknn_app_ctx.img_dma_buf.dma_buf_virt_addr;
        src_image.fd = rknn_app_ctx.img_dma_buf.dma_buf_fd;
        rknn_app_ctx.img_dma_buf.size = src_image.size;
        printf("free\n");
        if (ret != 0)
        {
            printf("read image fail! ret=%d img_path=%s\n", ret, img_path);
            deinit_post_process();

            ret = release_yolov5_model(&rknn_app_ctx);
            if (ret != 0)
            {
                printf("release_yolov5_model fail! ret=%d\n", ret);
            }
        
            if (src_image.virt_addr != NULL)
            {dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                            rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
            }  
        }
        object_detect_result_list od_results;
                
        ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);
        std::cout<< "<6>process_image_with_yolov5_v2 inference_yolov5_model ok "<< std::endl;
        if (ret != 0)
        {
            printf("init_yolov5_model fail! ret=%d\n", ret);
            deinit_post_process();

            ret = release_yolov5_model(&rknn_app_ctx);
            if (ret != 0)
            {
                printf("release_yolov5_model fail! ret=%d\n", ret);
            }
        
            if (src_image.virt_addr != NULL)
            {
                dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                            rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);                        
            }  
        }
        for (int i = 0; i < od_results.count; i++)
        {
            object_detect_result *det_result = &(od_results.results[i]);
            printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                det_result->box.left, det_result->box.top,
                det_result->box.right, det_result->box.bottom,
                det_result->prop);
            box[i][0] = det_result->box.left;
            box[i][1] = det_result->box.top;
            box[i][2] = det_result->box.right;
            box[i][3] = det_result->box.bottom;
        }
        if (src_image.virt_addr != NULL)
        {
            dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                        rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);                        
        } 
    }
    printf("<#>process_image_with_yolov5_v2 resize_images_in_folder result ret = %d.\n",ret);
    return ret;
     
}

static cv::Mat cropFromScaledCoordinates(const string& src_path, const string& scale_path, int left, int top, int right, int bottom) {
    // 读取原图和缩放图片
    cv::Mat src_image = cv::imread(src_path);
    cv::Mat scale_image = cv::imread(scale_path);
    
    if (src_image.empty()) {
        throw runtime_error("无法读取原图: " + src_path);
    }
    if (scale_image.empty()) {
        throw runtime_error("无法读取缩放图片: " + scale_path);
    }
    
    // 获取原图和缩放图片的尺寸
    int src_width = src_image.cols;
    int src_height = src_image.rows;
    int scale_width = scale_image.cols;
    int scale_height = scale_image.rows;
    
    // 计算宽高缩放比例
    double width_ratio = static_cast<double>(src_width) / scale_width;
    double height_ratio = static_cast<double>(src_height) / scale_height;
    double ratio = width_ratio>height_ratio? width_ratio :height_ratio;
    
    int left_scale = left;
    int top_scale = top;
    int right_scale = right;
    int bottom_scale = bottom;
    
    // 将缩放图片坐标映射到原图坐标
    int left_src = static_cast<int>(left_scale * ratio);
    int top_src = static_cast<int>(top_scale * ratio);
    int right_src = static_cast<int>(right_scale * ratio);
    int bottom_src = static_cast<int>(bottom_scale * ratio);
    
    // 确保坐标在图像范围内
    left_src = max(0, min(left_src, src_width - 1));
    top_src = max(0, min(top_src, src_height - 1));
    right_src = max(0, min(right_src, src_width - 1));
    bottom_src = max(0, min(bottom_src, src_height - 1));
    
    // 验证坐标有效性
    if (left_src >= right_src || top_src >= bottom_src) {
        throw runtime_error("无效的坐标范围!!");
    }
    
    // 从原图中截取对应区域
    cv::Rect roi(left_src, top_src, right_src - left_src, bottom_src - top_src);
    cv::Mat cropped_image = src_image(roi);
    
    return cropped_image;
}

static bool cropImage(const char* src_path, const char* transform_path, int left, int top, int right, int bottom) {
    // 读取原始图像
    cv::Mat image = cv::imread(src_path);
    if (image.empty()) {
        std::cerr << "Error: Could not read the image at " << src_path << std::endl;
        return false;
    }

    // 检查坐标是否有效
    if (left < 0 || top < 0 || right > image.cols || bottom > image.rows || 
        left >= right || top >= bottom) {
        std::cerr << "Error: Invalid coordinates for cropping" << std::endl;
        return false;
    }

    // 定义ROI (Region of Interest)
    cv::Rect roi(left, top, right - left, bottom - top);

    // 裁剪图像
    cv::Mat croppedImage = image(roi);

    // 保存裁剪后的图像
    if (!cv::imwrite(transform_path, croppedImage)) {
        std::cerr << "Error: Could not save the cropped image to " << transform_path << std::endl;
        return false;
    }

    return true;
}

/**
    从原图中截图目标

     步 骤
    （1）缩放scale_path  
    （2）目标检测数据scale_path    
    （3）截图至transform_path
    （4）清理文件
 * @param src_path 原图path
 * @param src_path 裁减图path
 * @return 成功返回0 失败返回1
 */
 static int transform_pic_my(const char * src_path, char * transform_path){
    std::cout << "transform_pic_my : "<< src_path<<std::endl;
    snprintf(scale_path,1024,"%s/%s_%d.jpg",SCALE_PATH,"hd_scale",scale_index++);
    int tmp [5][4] = {0};
    int ret =  process_image_with_yolov5_v2(src_path,scale_path,tmp,rknn_app_ctx);
    std::cout << "process_image_with_yolov5_v2 ret = " << ret <<std::endl;
    int left = tmp[0][0];
    int top = tmp[0][1];
    int right = tmp[0][2];
    int bottom = tmp[0][3];
    if (left!=0 && top !=0 && right !=0 && bottom !=0)
    {
            std::cout << "before cropImage "<< src_path <<"["<<left<<","<<top<<","<<right<<","<<bottom<<"]"<<std::endl;
            bool result = false;
            try{
                cv::Mat r = cropFromScaledCoordinates(src_path,scale_path,left, top, right, bottom);
                    if (!cv::imwrite(transform_path, r)) {
                    std::cerr << "Error: Could not save the cropped image to " << transform_path << std::endl;
                    return 3;
                }
                std::cout << "cropImage success !!!!! " << transform_path  << std::endl;
                result = true;
            } catch (const std::exception& e) {
                std::cerr << "cropImage fail!!!!!错误: " << e.what() << std::endl;
            }
        return result?0:2;
    }
    deleteFile(scale_path);
    return 1;
 }  
   
 /*-------------------------------------------
                默认
-------------------------------------------*/
static void sig_proc(int signo) {
	LOG_INFO("received signo %d \n", signo);
	g_main_run_ = 0;
}

static const char short_options[] = "c:a:l:";
static const struct option long_options[] = {{"config", required_argument, NULL, 'c'},
                                             {"aiq_file", no_argument, NULL, 'a'},
                                             {"log_level", no_argument, NULL, 'l'},
                                             {"help", no_argument, NULL, 'h'},
                                             {0, 0, 0, 0}};

static void usage_tip(FILE *fp, int argc, char **argv) {
	fprintf(fp,
	        "Usage: %s [options]\n"
	        "Version %s\n"
	        "Options:\n"
	        "-c | --config      rkipc ini file, default is "
	        "/userdata/rkipc.ini, need to be writable\n"
	        "-a | --aiq_file    aiq file dir path, default is /etc/iqfiles\n"
	        "-l | --log_level   log_level [0/1/2/3], default is 2\n"
	        "-h | --help        for help \n\n"
	        "\n",
	        argv[0], "V1.0");
}

void rkipc_get_opt(int argc, char *argv[]) {
	for (;;) {
		int idx;
		int c;
		c = getopt_long(argc, argv, short_options, long_options, &idx);
		if (-1 == c)
			break;
		switch (c) {
		case 0: /* getopt_long() flag */
			break;
		case 'c':
			rkipc_ini_path_ = optarg;
			break;
		case 'a':
			rkipc_iq_file_path_ = optarg;
			break;
		case 'l':
			rkipc_log_level = atoi(optarg);
			break;
		case 'h':
			usage_tip(stdout, argc, argv);
			exit(EXIT_SUCCESS);
		default:
			usage_tip(stderr, argc, argv);
			exit(EXIT_FAILURE);
		}
	}
}

int main(int argc, char **argv)
{    
    double uptime_seconds = getUptimeSeconds();
    if (uptime_seconds < 15) {
        if (deleteFile("/userdata/watchdog.pid")) {
        std::cout << "删除看门狗PID文件成功" << std::endl;
        } else {
            std::cout << "删除看门狗PID文件失败或文件不存在" << std::endl;
        }
    }

    // start_watchdog();
    std::cout << "主程序开始运行,PID: " << getpid() << std::endl;
    std::cout << "app_version:" << app_version << std::endl;
    const char* path = "/userdata/jpeg";
	LOG_DEBUG("main begin\n");
	rkipc_version_dump();
	signal(SIGINT, sig_proc);
	signal(SIGTERM, sig_proc);

	recv_callback_func func = {qjy_uart_parser, hd_uart_recv};

	rkipc_get_opt(argc, argv);
	LOG_INFO("rkipc_ini_path_ is %s, rkipc_iq_file_path_ is %s, rkipc_log_level "
	         "is %d~~~~~~~~~~~~\n",
	         rkipc_ini_path_, rkipc_iq_file_path_, rkipc_log_level);
	
    if(access(path, F_OK) == 0) {
        LOG_INFO("Directory exists.\n");
    } else {
        if( mkdir(path, 0755) == -1 ){
			LOG_ERROR("create folder fail\n");
		}
    }

    ensure_path_exists(SCALE_PATH);

	if(access("/userdata/update_ota.tar", F_OK) == 0)
	{
		if(remove("/userdata/update_ota.tar") == 0)
		{
			LOG_INFO("remove ota file success!\n");
		}else{
			LOG_ERROR("remove ota file failed!\n");
		}
	}

    rk_param_init(rkipc_ini_path_);
	rk_isp_init(0, rkipc_iq_file_path_);
	rk_isp_set_from_ini(1);
	RK_MPI_SYS_Init();
	qjy_uart_init(&func, HD_CAMERA_ADDR);
	gsensor_init(HD_CAMERA_ADDR==1?0:1);
    usleep(1000);
    restore_sensor();
	qjy_photo_init();
	heat_pwm_init();

    const char *image_tmp_path = "/userdata/tmp_images_path";
    const char *images_dir_path = "/userdata/images_dir_path";

    delete_specified_folder(image_tmp_path);    
    delete_specified_folder(images_dir_path);

    ensure_path_exists(image_tmp_path);
    ensure_path_exists(images_dir_path);

    if (HD_CAMERA_ADDR == 2)
    {
        std::string model_path_std;
    
        std::string userdata_rknn_path_new = findRknnFile("/userdata");
        std::string userdata_rknn_path_old = findRknnFile("/oem/usr/share");

    if (!userdata_rknn_path_new.empty()){
        if (deleteFile(userdata_rknn_path_old)){
            std::cout << "权重文件已删除" << std::endl;
        } else {
        std::cout << "文件删除失败或文件不存在" << std::endl;
        }
        
        if (moveFile(userdata_rknn_path_new, "/oem/usr/share/")) {
            std::cout << "移动成功" << std::endl;
            model_path_std = findRknnFile("/oem/usr/share");
        } else {
            std::cerr << "移动失败" << std::endl;
        }
    } else {
        model_path_std = userdata_rknn_path_old;
    }

    if (HD_CAMERA_ADDR == 2){
        const char* model_path = model_path_std.c_str();
        int ret;
        memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
        init_post_process();
        ret = init_yolov5_model(model_path, &rknn_app_ctx);
        if (ret != 0)
        {
            printf("init_yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
            deinit_post_process();
        
            ret = release_yolov5_model(&rknn_app_ctx);
            if (ret != 0)
            {
                printf("release_yolov5_model fail! ret=%d\n", ret);
            }
            return 0;
        }
    }
}

    hd_uart_init(HD_CAMERA_ADDR, images_dir_path, app_version, action_id_collect, on_event,transform_pic_my);

    while (g_main_run_) {
		usleep(1000 * 1000);
	}

	rk_param_deinit();
    hd_uart_deinit();
	qjy_photo_deinit();
	rk_isp_deinit(0);
	RK_MPI_SYS_Exit();
	//pthread_sem_deinit();
	qjy_uart_deinit();
	gsensor_deinit();
    return 0;
}




