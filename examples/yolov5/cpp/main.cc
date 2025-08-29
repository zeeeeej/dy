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

std::string app_version = "V0.4";
static int pic_id = 0;
static int pic_action_id = 0; // 用于标识拍照的动作ID

int addr_biu = 2;

static uint8_t door_status = 2;  // 默认日志级别为INFO

static int g_main_run_ = 1;
char *rkipc_ini_path_ = NULL;
char *rkipc_iq_file_path_ = NULL;


static int zero_count = 0;
static int last_reported_angle = -9999;


namespace mydata {
    std::string action_id_txt_name = "/userdata/action_id.txt";
}



void action_id_collect(uint8_t status, const char *action_id){
    // door_status = status;
    // if (status==1){ 
    //     if (!action_id || action_id[0] == '\0') return;  // 防止空指针写进文件
    //     std::ofstream outfile(mydata::action_id_txt_name); 
    //     if (!outfile.is_open()) return;
    //     outfile << action_id << std::endl;
    // } else if (status == 0)
    // {
    //     restore_sensor();
    // }
}


void *on_event(int event_id, void *event_value, size_t event_value_size) {
    return NULL;
}




////////////////

bool x_cp_file(const char* src_path, const char* dest_path) {
    // 打开源文件（二进制模式）
    std::ifstream src(src_path, std::ios::binary);
    if (!src.is_open()) {
        std::cerr << "无法打开源文件 '" << src_path << "': " << strerror(errno) << std::endl;
        return false;
    }

    // 打开目标文件（二进制模式）
    std::ofstream dest(dest_path, std::ios::binary);
    if (!dest.is_open()) {
        std::cerr << "无法打开目标文件 '" << dest_path << "': " << strerror(errno) << std::endl;
        src.close();
        return false;
    }

    // 复制文件内容
    dest << src.rdbuf();

    // 检查是否复制成功
    if (!dest.good()) {
        std::cerr << "复制文件时发生错误" << std::endl;
        src.close();
        dest.close();
        return false;
    }

    // 关闭文件
    src.close();
    dest.close();

    return true;
}

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
int process_image_with_yolov5_v2(const std::string& src_path,const std::string& scale_path, int box[5][4], rknn_app_context_t& rknn_app_ctx) {
    int ret = 0;
    printf("<$>process_image_with_yolov5_v2 %s %s\n", src_path,scale_path);
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


// 新添加的函数
/**
 * @param src_path 原始图片
 * @param dst_path 生成的目标图片
 */
int process_image_with_yolov5(const std::string& src_path, const std::string& dst_path, 
                                    const std::string& model_path, int target_width = 960) {
    // // 1. 读取原始图像并等比例缩放
    // cv::Mat src_img = cv::imread(src_path);
    // if (src_img.empty()) {
    //     throw std::runtime_error("无法加载图像: " + src_path);
    // }

    // // 计算缩放比例
    // double scale = static_cast<double>(target_width) / src_img.cols;
    // cv::Mat scaled_img;
    // cv::resize(src_img, scaled_img, cv::Size(), scale, scale, cv::INTER_LINEAR);

    // // 保存缩放后的图像到临时文件
    // std::string temp_dir = fs::path(dst_path).parent_path().string();
    // std::string scale_path = temp_dir + "/temp_scaled.jpg";
    // if (!cv::imwrite(scale_path, scaled_img)) {
    //     throw std::runtime_error("无法保存缩放后的图像: " + scale_path);
    // }

    // // 2. 准备RKNN推理
    // // rknn_app_context_t rknn_app_ctx;
    // // memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    // // init_post_process();

    // // int ret = init_yolov5_model(model_path.c_str(), &rknn_app_ctx);
    // // if (ret != 0) {
    // //     fs::remove(scale_path);
    // //     throw std::runtime_error("init_yolov5_model fail! ret=" + std::to_string(ret));
    // // }

    // // 3. 准备输入图像
    // image_buffer_t src_image;
    // memset(&src_image, 0, sizeof(image_buffer_t));
    // src_image.width = scaled_img.cols;
    // src_image.height = scaled_img.rows;
    // src_image.format = IMAGE_FORMAT_RGB888;
    // src_image.size = scaled_img.total() * scaled_img.elemSize();
    // src_image.virt_addr = (unsigned char*)malloc(src_image.size);
    
    // // 将OpenCV Mat转换为RGB格式
    // cv::Mat rgb_img;
    // cv::cvtColor(scaled_img, rgb_img, cv::COLOR_BGR2RGB);
    // memcpy(src_image.virt_addr, rgb_img.data, src_image.size);

    // // 4. 执行推理
    // object_detect_result_list od_results;
    // ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);
    // if (ret != 0) {
    //     free(src_image.virt_addr);
    //     release_yolov5_model(&rknn_app_ctx);
    //     fs::remove(scale_path);
    //     throw std::runtime_error("inference_yolov5_model fail! ret=" + std::to_string(ret));
    // }

    // // 5. 处理检测结果并裁剪原始图像
    // if (od_results.count > 0) {
    //     // 取置信度最高的检测结果
    //     object_detect_result* best_result = &od_results.results[0];
    //     for (int i = 1; i < od_results.count; i++) {
    //         if (od_results.results[i].prop > best_result->prop) {
    //             best_result = &od_results.results[i];
    //         }
    //     }

    //     // 将检测框坐标映射回原始图像
    //     int x1 = static_cast<int>(best_result->box.left / scale);
    //     int y1 = static_cast<int>(best_result->box.top / scale);
    //     int x2 = static_cast<int>(best_result->box.right / scale);
    //     int y2 = static_cast<int>(best_result->box.bottom / scale);

    //     // 确保坐标在图像范围内
    //     x1 = std::max(0, x1);
    //     y1 = std::max(0, y1);
    //     x2 = std::min(src_img.cols - 1, x2);
    //     y2 = std::min(src_img.rows - 1, y2);

    //     // 裁剪图像
    //     cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
    //     cv::Mat cropped_img = src_img(roi);

    //     // 保存裁剪后的图像
    //     if (!cv::imwrite(dst_path, cropped_img)) {
    //         free(src_image.virt_addr);
    //         release_yolov5_model(&rknn_app_ctx);
    //         fs::remove(scale_path);
    //         throw std::runtime_error("无法保存裁剪后的图像: " + dst_path);
    //     }
    // } else {
    //     free(src_image.virt_addr);
    //     release_yolov5_model(&rknn_app_ctx);
    //     fs::remove(scale_path);
    //     throw std::runtime_error("未检测到任何目标");
    // }

    // // 6. 清理资源
    // free(src_image.virt_addr);
    // release_yolov5_model(&rknn_app_ctx);
    // fs::remove(scale_path);

    // return dst_path;
    return 0;
}
  rknn_app_context_t rknn_app_ctx;


  using namespace cv;
using namespace std;
  cv::Mat cropFromScaledCoordinates(const string& src_path, 
                             const string& scale_path, 
                             int left, int top, int right, int bottom) {
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
    
    int left_scale = left;
    int top_scale = top;
    int right_scale = right;
    int bottom_scale = bottom;
    
    // 将缩放图片坐标映射到原图坐标
    int left_src = static_cast<int>(left_scale * width_ratio);
    int top_src = static_cast<int>(top_scale * height_ratio);
    int right_src = static_cast<int>(right_scale * width_ratio);
    int bottom_src = static_cast<int>(bottom_scale * height_ratio);
    
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

  bool cropImage(const char* src_path, const char* transform_path, 
               int left, int top, int right, int bottom) {
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

static int scale_index = 0;
/**
 * 
 * @param src_path 原图path
 * @param src_path 裁减图path
 * @return 成功返回0 失败返回1
 * int(*transform_pic)(const char *, char *)
 */
 int transform_pic_my(const char * src_path, char * transform_path){
    // const std::string& src_path, int box[5][4], rknn_app_context_t& rknn_app_ctx 111
     std::cout << "transform_pic_my =>"<< src_path<<std::endl;
     char  salce_path[1024];
     snprintf(salce_path,1024,"/userdata/%s_%05d.jpg","hd_scale",scale_index++);
    int tmp [5][4] = {0};
    //if(rknn_app_ctx){
        int ret =  process_image_with_yolov5_v2(src_path,salce_path,tmp,rknn_app_ctx);
        std::cout << "process_image_with_yolov5_v2 ret = " << ret <<std::endl;
        for (size_t i = 0; i < 5; i++)
        {
            std::cout << i <<"<----" << std::endl;

            for (size_t j = 0; j < 4; j++)
            {
                  std::cout <<  tmp[i][j] << std::endl;
            }
            
              
        }
           std::cout << "before cropImage  =>"<< src_path<<std::endl;
        int left = tmp[0][0];
        int top = tmp[0][1];
        int right = tmp[0][2];
        int bottom = tmp[0][3];
        if (left!=0 && top !=0 && right !=0 && bottom !=0)
        {
                // int sacle = 2; // todo 计算scale
                // bool result =cropImage (src_path,transform_path,left*sacle,top*sacle,right*sacle,bottom*sacle);
                // if (result)
                // {
                //     std::cout << "cropImage success !"  << std::endl;
                // }
                // else{
                //      std::cout << "cropImage false !"  << std::endl;
                // }
                bool result = false;
                try{
                    cv::Mat r = cropFromScaledCoordinates(src_path,salce_path,left, top, right, bottom);
                     if (!cv::imwrite(transform_path, r)) {
                        std::cerr << "Error: Could not save the cropped image to " << transform_path << std::endl;
                        return 3;
                    }
                    std::cout << "cropImage success !!!!!"  << std::endl;
                    result = true;
                } catch (const std::exception& e) {
                    std::cerr << "错误: " << e.what() << std::endl;
                }
            return result?0:2;
        }
        


        return 1;
    // }else{
    //     return 0;
    // }


    // process_image_with_yolov5(
    //     src_path,transform_path,"",960
    // );   

    // // （1）复制副本bak_path
    // char  bak_path[1024];
    // if(x_cp_file(src_path,bak_path)){
    //     return 1;
    // }

    // // （2）缩放    
    //  int max_length = 960;
    // std::string extension = bak_path.extension().string();
    // if (extension == ".jpg" || extension == ".png" || extension == ".bmp") {
    //     cv::Mat img = cv::imread(bak_path);
    //     if (img.empty()) {
    //         st d::cerr << "无法读取图片: " << bak_path << std::endl;
    //         continue;
    //     }

    //     int width = img.cols;
    //     int height = img.rows;
    //     std::cout << "处理图片: " << bak_path << " (原始尺寸: " << width << "x" << height << ")" << std::endl;
    //     int long_side = std::max(width, height);

    //     // 如果已经小于等于 max_length，则跳过
    //     if (long_side <= max_length) continue;

    //     // 计算缩放比例
    //     double scale = static_cast<double>(max_length) / long_side;
    //     int new_width = static_cast<int>(width * scale);
    //     int new_height = static_cast<int>(height * scale);

    //     cv::Mat resized;
    //     cv::resize(img, resized, cv::Size(new_width, new_height));

    //     // 覆盖保存
    //     if (!cv::imwrite(bak_path, resized)) {
    //         std::cerr << "保存失败: " << bak_path << std::endl;
    //         return 1;
    //     } else {
    //         std::cout << "处理完成: " << bak_path << std::endl;
    //     }
    // }
    
    // // (3)对bak_path进行目标检测，得到目标检测数据。

    // // 根据目标数据从src_path中裁剪

    // // 将裁减结果放到transform_path

    // // 删除副本文件

    // return 1;
 }    



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


void writeStringToFileAfterDelay(const std::string& file_path, const std::string& content, int delay_seconds) {
    // 延迟 delay_seconds 秒
    std::this_thread::sleep_for(std::chrono::seconds(delay_seconds));

    // 打开文件（覆盖写入）
    std::ofstream outfile(file_path, std::ios::out);
    if (!outfile) {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return;
    }

    // 写入内容
    outfile << content;
    outfile.close();

    std::cout << "写入完成: " << file_path << std::endl;
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
	
	qjy_uart_init(&func, addr_biu);
	gsensor_init(addr_biu==1?0:1);
    usleep(1000);
    restore_sensor();
	qjy_photo_init();
	heat_pwm_init();



/*--------------判断图片路径是否存在并创建---------------------*/

    const char *image_tmp_path = "/userdata/tmp_images_path";
    const char *images_dir_path = "/userdata/images_dir_path";

    delete_specified_folder(image_tmp_path);
    delete_specified_folder(images_dir_path);

    ensure_path_exists(image_tmp_path);
    ensure_path_exists(images_dir_path);

    if (addr_biu == 2)
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


     if (addr_biu == 2){
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

    hd_uart_init(addr_biu, images_dir_path, app_version.c_str(), action_id_collect, on_event,transform_pic_my);

    
    
// /*--------------陀螺仪检测并拍照------------------------------*/

//     ThreadSafeSet<std::string> photo_names(20);
//     ThreadSafeSet<std::string> action_id_record(20);

//     bool door_closed_reported = false;

//     std::thread t1([&image_tmp_path, &images_dir_path, &photo_names, &action_id_record, &door_closed_reported]() {
//         while (g_main_run_) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(50));

//             int angle1 = get_angle();  
//             float result = tly_detect1(angle1);
        

//             if (result <= 0) {
//                 zero_count++;
                
//                 if (zero_count >= 40 ) {
//                     last_reported_angle = 0;
//                 }
//             } else {
//                 zero_count = 0;  // 非0则清零计数
               
//                 if (result != last_reported_angle) {
//                 std::cout << "检测到陀螺仪角度!!!!!!!!!!: " << result << std::endl;   
//                 }
//                 last_reported_angle = result;
//             }
          
//             if (last_reported_angle >= 50.0f && !door_closed_reported && door_status == 1) {

//                 std::cout << "检测到陀螺仪角度*************: " << last_reported_angle << std::endl;
               
//                 auto now = std::chrono::system_clock::now();
            
//                 std::time_t time_now = std::chrono::system_clock::to_time_t(now);   
            
//                 auto duration = now.time_since_epoch();
//                 auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
              
//                 std::ostringstream oss;
             
//                 pic_id = (pic_id + 1) % 0x41;

//                 oss << std::setfill('0') << std::setw(3) << millis.count();  
//                 oss << "_0_" << std::to_string(last_reported_angle) <<"_" << "1" << "_" << time_now << "_" << pic_id << ".jpg";
//                 std::string image_biu_name_path = oss.str();
//                 std::string image_biu_name_path_change = "gai1_0_" + std::to_string(last_reported_angle) + "_" + std::to_string(time_now) + "_" + std::to_string(pic_id) + ".jpg";

                
//                 if (take_photo(2, image_tmp_path,image_biu_name_path)) {
//                     std::string image_biu_path = std::string(image_tmp_path) + "/" + image_biu_name_path;
//                     std::string image_biu_path_change = std::string(image_tmp_path) + "/" + image_biu_name_path_change;
//                     std::this_thread::sleep_for(std::chrono::milliseconds(200));
//                     if (compressImageToTargetSize(image_biu_path, image_biu_path_change, 300)) {
//                         photo_names.insert(image_biu_path_change);
//                         std::filesystem::remove(image_biu_path); 
//                     } else {
//                         createBlankImage(image_biu_path_change);
//                         photo_names.insert(image_biu_path_change);
//                         std::filesystem::remove(image_biu_path); 
//                     }
   
//                 };

//                 // writeStringToFileAfterDelay("/userdata/action_id.txt", "1748939045000", 0);
// 				// std::this_thread::sleep_for(std::chrono::milliseconds(500));
//                 last_reported_angle = 1; 
//                 door_closed_reported = true; // 关门后设置为true，防止重复报告
               
//             } else if (last_reported_angle <= 20.0f && door_closed_reported && door_status == 0) {
//                 door_closed_reported = false;
//                 std::string action_id = read_txt_file(mydata::action_id_txt_name);
//                 // std ::cout << "读取到的action_id: " << action_id << std::endl;
//                 clearFile(mydata::action_id_txt_name);
//                 if (!action_id.empty()) {
//                     std::string action_id_image_path_finall = std::string(images_dir_path) + "/" + action_id;
//                     ensure_path_exists(action_id_image_path_finall.c_str());
//                     movePhotos(photo_names, action_id_image_path_finall, action_id_record);
//                     delete_folder_contents_only(image_tmp_path);
//                     action_id_record.clear();
//                 } 
//             }
//         }
//     });


    // while (g_main_run_) {
	// 	// delete_oldest_folders(images_dir_path, 50);
	// 	std::this_thread::sleep_for(std::chrono::minutes(1));
	// }
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




