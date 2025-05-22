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

static int pic_id = 0;

namespace mydata {
    std::string action_id_txt_name = "/userdata/action_id.txt";
}



void action_id_collect(const unsigned char *action_id, size_t action_id_size){
    std::ofstream outfile(mydata::action_id_txt_name); 
    if (!outfile) return;

    for (size_t i = 0; i < action_id_size; ++i) {
        outfile << std::hex << std::setw(2) << std::setfill('0') << (int)action_id[i];
        if (i != action_id_size - 1) outfile << "_";
    }
    outfile << std::endl;

    outfile.close();
}



static int g_main_run_ = 1;
char *rkipc_ini_path_ = NULL;
char *rkipc_iq_file_path_ = NULL;

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

    const char* path = "/userdata/jpeg";
	LOG_DEBUG("main begin\n");
	rkipc_version_dump();
	signal(SIGINT, sig_proc);
	signal(SIGTERM, sig_proc);

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
	
	qjy_uart_init((void*)qjy_uart_parser);
	gsensor_init();
	qjy_photo_init();
	heat_pwm_init();



    const char *model_path = "/oem/usr/share/one_category_full.rknn";
    
/*--------------判断图片路径是否存在并创建---------------------*/

    const char *image_tmp_path = "/userdata/tmp_images_path";
    ensure_path_exists(image_tmp_path);

    const char *images_dir_path = "/userdata/images_dir_path";
    ensure_path_exists(images_dir_path);



/*--------------陀螺仪检测并拍照------------------------------*/
    // float angle1 = 20.0f;

    std::set<std::string> photo_names;
    std::set<std::string>* photo_names_ptr = &photo_names;
    
    std::set<std::string> action_id_record;
    std::set<std::string>* action_id_record_ptr = &action_id_record; 


    std::thread t1([&image_tmp_path, &images_dir_path, &photo_names_ptr, &action_id_record_ptr]() {
        while (true) {
            int angle1 = get_angle();
            float result = tly_detect(angle1);
            std::cout << "检测到陀螺仪角度: " << result << std::endl;
          
            if (result >= 20.0f) {
                auto now = std::chrono::system_clock::now();
                // 获取time_t格式（秒）
                std::time_t time_now = std::chrono::system_clock::to_time_t(now);   
                // 线程安全地转换为tm结构
                // std::tm tm_now;
                // localtime_r(&time_now, &tm_now);
                // 计算毫秒部分
                auto duration = now.time_since_epoch();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
                // 构建路径字符串
                std::ostringstream oss;
                // oss << image_tmp_path;
                // if (!std::string(image_tmp_path).empty() && std::string(image_tmp_path).back() != '/') {
                //     oss << "/";
                // }
                // oss << std::put_time(&tm_now, "%Y_%m_%d_%H_%M_%S");
                // oss << "_" << std::setfill('0') << std::setw(3) << millis.count();  // 补零到3位
                // oss << ".jpg";
                pic_id = (pic_id + 1) & 0xFF;

                oss << std::setfill('0') << std::setw(3) << millis.count();  
                oss << "_" << time_now << "_" << pic_id << ".jpg";
                std::string image_biu_name_path = oss.str();
                
                if (take_photo(2, image_tmp_path,image_biu_name_path)) {
                    std::string image_biu_path = std::string(image_tmp_path) + "/" + image_biu_name_path;
                    photo_names_ptr->insert(image_biu_path);
                };
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            } else {
                std::string action_id = read_txt_file(mydata::action_id_txt_name);
                if (!action_id.empty()) {
                    std::string action_id_image_path_finall = std::string(images_dir_path) + "/" + action_id;
                    movePhotos(*photo_names_ptr, action_id_image_path_finall, *action_id_record_ptr);
                    clearFile(mydata::action_id_txt_name);
                } else {
                    std::cout << "没有action_id!!" << std::endl;
                }
            }
        }
    });



    int ret;
    rknn_app_context_t rknn_app_ctx;
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
    }

    const char* txt_dir = "/userdata/txt_dir_path";
    ensure_path_exists(txt_dir);
   


    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (!action_id_record.empty()) {
            for (const std::string& action_id_image_path : action_id_record) {
                std::string action_id_biu = std::filesystem::path(action_id_image_path).filename().string();
                std::vector<std::string> frames = get_image_paths(action_id_image_path);
/*--------------事件id检测的文件夹------------------------------*/
                if (!frames.empty()) {
                    for (const std::string& img_path : frames) {
                            
                            std::cout << "图片路径: " << img_path << std::endl;

                            std::string frame_id = std::filesystem::path(img_path).stem().string();

                            image_buffer_t src_image;
                            memset(&src_image, 0, sizeof(image_buffer_t));
                            ret = read_image(img_path.c_str(), &src_image);

                        #if defined(RV1106_1103) 
                            //RV1106 rga requires that input and output bufs are memory allocated by dma
                            ret = dma_buf_alloc(RV1106_CMA_HEAP_PATH, src_image.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                                            (void **) & (rknn_app_ctx.img_dma_buf.dma_buf_virt_addr));
                            memcpy(rknn_app_ctx.img_dma_buf.dma_buf_virt_addr, src_image.virt_addr, src_image.size);
                            dma_sync_cpu_to_device(rknn_app_ctx.img_dma_buf.dma_buf_fd);
                            free(src_image.virt_addr);
                            src_image.virt_addr = (unsigned char *)rknn_app_ctx.img_dma_buf.dma_buf_virt_addr;
                            src_image.fd = rknn_app_ctx.img_dma_buf.dma_buf_fd;
                            rknn_app_ctx.img_dma_buf.size = src_image.size;
                        #endif

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
                                {
                                #if defined(RV1106_1103) 
                                        dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                                                rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
                                #else
                                        free(src_image.virt_addr);
                                #endif
                                    }  
                            }
                        
                            object_detect_result_list od_results;
                        
                            ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);
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
                                #if defined(RV1106_1103) 
                                        dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                                                rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
                                #else
                                        free(src_image.virt_addr);
                                #endif
                                    }  
                            }
                        

                            for (int i = 0; i < od_results.count; i++)
                            {
                                object_detect_result *det_result = &(od_results.results[i]);
                                printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                                    det_result->box.left, det_result->box.top,
                                    det_result->box.right, det_result->box.bottom,
                                    det_result->prop);
                                int x1 = det_result->box.left;
                                int y1 = det_result->box.top;
                                int x2 = det_result->box.right;
                                int y2 = det_result->box.bottom;

                                std::string frame_number_str = action_id_biu + "_" + frame_id + "_" + std::to_string(i);


                                if (od_results.count == 1 && std::strcmp(coco_cls_to_name(det_result->cls_id), "full")==0 && det_result->prop >= 0.9){
                                    std::string txt1_name_path = std::string(txt_dir) + "/" + action_id_biu + "_1" + ".txt";
                                    std::ofstream outfile(txt1_name_path, std::ios::app); 
                                    if (outfile.is_open()) {
                                        outfile << frame_number_str << " "
                                                << det_result->cls_id << " "
                                                << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
                                        outfile.close();  // 关闭文件
                                        std::cout << "写入成功！" << std::endl;
                                    } else {
                                        std::cout << "无法打开文件！" << std::endl;
                                    }
                                }

                                if (od_results.count == 2 && std::strcmp(coco_cls_to_name(det_result->cls_id), "full")==0 && det_result->prop >= 0.5){
                                    std::string txt2_name_path = std::string(txt_dir) + "/" + action_id_biu + "_2" + ".txt";
                                    std::ofstream outfile(txt2_name_path, std::ios::app); 
                                    if (outfile.is_open()) {
                                        outfile << frame_number_str << " "
                                                << det_result->cls_id << " "
                                                << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
                                        outfile.close();  // 关闭文件
                                        std::cout << "写入成功！" << std::endl;
                                    } else {
                                        std::cout << "无法打开文件！" << std::endl;
                                    }        
                                }

                            }
                            
               

                    }
                    
            }


            std::string txt1_name_path_result = std::string(txt_dir) + "/" + action_id_biu + "_1" + ".txt";

            std::string txt2_name_path_result = std::string(txt_dir) + "/" + action_id_biu + "_2" + ".txt";

            std::string crop_img_dirs = "/userdata/crop_images";

            std::string final_result;
            if (file_exists_and_not_empty(txt2_name_path_result)){
                final_result = analyse_two(txt2_name_path_result);
                int line_n = 10;
                std::string crop_img_path = crop_img_dirs + "/" + action_id_biu;
                if (process_last_n_lines(txt2_name_path_result, crop_img_path, line_n)) {
                    std::cout << "有截图保存成功" << std::endl;
                } else {
                    std::cout << "没有任何截图保存成功" << std::endl;
                }
            } else {
                final_result = analyse_one(txt1_name_path_result);
                int line_n = 5;
                std::string crop_img_path = crop_img_dirs + "/" + action_id_biu;
                if (process_last_n_lines(txt1_name_path_result, crop_img_path, line_n)) {
                    std::cout << "有截图保存成功" << std::endl;
                } else {
                    std::cout << "没有任何截图保存成功" << std::endl;
                }
            }
            // if (file_exists_and_not_empty(txt1_name_path_result)) 

            }
        }
    
       
    } 

    while (g_main_run_) {
		usleep(1000 * 1000);
	}


	rk_param_deinit();
	qjy_photo_deinit();

	rk_isp_deinit(0);
	
	RK_MPI_SYS_Exit();
	
	pthread_sem_deinit();
	qjy_uart_deinit();
	gsensor_deinit();
      
    return 0;
}




