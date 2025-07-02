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

std::string app_version = "V1.1";
static int pic_id = 0;
static int pic_action_id = 0; // 用于标识拍照的动作ID

int addr_biu = 1;

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
    door_status = status;
    if (status==1){ 
        if (!action_id || action_id[0] == '\0') return;  // 防止空指针写进文件
        std::ofstream outfile(mydata::action_id_txt_name); 
        if (!outfile.is_open()) return;
        outfile << action_id << std::endl;
    }  
}


void *on_event(int event_id, void *event_value, size_t event_value_size) {
    return NULL;
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
	gsensor_init(0);
	qjy_photo_init();
	heat_pwm_init();




/*--------------判断图片路径是否存在并创建---------------------*/

    const char *image_tmp_path = "/userdata/tmp_images_path";
    const char *images_dir_path = "/userdata/images_dir_path";

    delete_specified_folder(image_tmp_path);
    delete_specified_folder(images_dir_path);

    ensure_path_exists(image_tmp_path);
    ensure_path_exists(images_dir_path);

	hd_uart_init(addr_biu, images_dir_path, action_id_collect, on_event);

    
/*--------------陀螺仪检测并拍照------------------------------*/

    ThreadSafeSet<std::string> photo_names(10);
    ThreadSafeSet<std::string> action_id_record(10);

    bool door_closed_reported = false;

    std::thread t1([&image_tmp_path, &images_dir_path, &photo_names, &action_id_record, &door_closed_reported]() {
        while (g_main_run_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            int angle1 = get_angle();  
            float result = tly_detect1(angle1);
        

            if (result <= 0) {
                zero_count++;
                
                if (zero_count >= 40 ) {
                    last_reported_angle = 0;
                }
            } else {
                zero_count = 0;  // 非0则清零计数
               
                if (result != last_reported_angle) {
                std::cout << "检测到陀螺仪角度!!!!!!!!!!: " << result << std::endl;   
                }
                last_reported_angle = result;
            }
          
            if (last_reported_angle >= 50.0f && !door_closed_reported && door_status == 1) {

                std::cout << "检测到陀螺仪角度*************: " << last_reported_angle << std::endl;
               
                auto now = std::chrono::system_clock::now();
            
                std::time_t time_now = std::chrono::system_clock::to_time_t(now);   
            
                auto duration = now.time_since_epoch();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
              
                std::ostringstream oss;
             
                pic_id = (pic_id + 1) % 0x41;

                oss << std::setfill('0') << std::setw(3) << millis.count();  
                oss << "_" << "1" << "_" << time_now << "_" << pic_id << ".jpg";
                std::string image_biu_name_path = oss.str();
                std::string image_biu_name_path_change = "gai_1_" + std::to_string(time_now) + "_" + std::to_string(pic_id) + ".jpg";

                
                if (take_photo(2, image_tmp_path,image_biu_name_path)) {
                    std::string image_biu_path = std::string(image_tmp_path) + "/" + image_biu_name_path;
                    std::string image_biu_path_change = std::string(image_tmp_path) + "/" + image_biu_name_path_change;
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    if (compressImageToTargetSize(image_biu_path, image_biu_path_change, 300)) {
                        photo_names.insert(image_biu_path_change);
                        std::filesystem::remove(image_biu_path); 
                    } else {
                        createBlankImage(image_biu_path_change);
                        photo_names.insert(image_biu_path_change);
                        std::filesystem::remove(image_biu_path); 
                    }
   
                };

                // writeStringToFileAfterDelay("/userdata/action_id.txt", "1748939045000", 0);
				// std::this_thread::sleep_for(std::chrono::milliseconds(500));
                last_reported_angle = 1; 
                door_closed_reported = true; // 关门后设置为true，防止重复报告
               
            } else if (last_reported_angle <= 20.0f && door_closed_reported && door_status == 0) {
                door_closed_reported = false;
                std::string action_id = read_txt_file(mydata::action_id_txt_name);
                // std ::cout << "读取到的action_id: " << action_id << std::endl;
                clearFile(mydata::action_id_txt_name);
                if (!action_id.empty()) {
                    std::string action_id_image_path_finall = std::string(images_dir_path) + "/" + action_id;
                    ensure_path_exists(action_id_image_path_finall.c_str());
                    movePhotos(photo_names, action_id_image_path_finall, action_id_record);
                    delete_folder_contents_only(image_tmp_path);
                    action_id_record.clear();
                } 
           
               
              
            }
        }
    });


    while (g_main_run_) {
		delete_oldest_folders(images_dir_path, 10);
		std::this_thread::sleep_for(std::chrono::minutes(1));
	}


	rk_param_deinit();
	qjy_photo_deinit();

    hd_uart_deinit();


	rk_isp_deinit(0);
	
	RK_MPI_SYS_Exit();
	
	pthread_sem_deinit();
	qjy_uart_deinit();
	gsensor_deinit();
      
    return 0;
}




