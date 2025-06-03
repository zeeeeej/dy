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

static int pic_id = 0;
static int pic_action_id = 0; // 用于标识拍照的动作ID

int addr_biu = 1;

namespace mydata {
    std::string action_id_txt_name = "/userdata/action_id.txt";
}



void action_id_collect(const char *action_id){
    if (!action_id || action_id[0] == '\0') return;  // 防止空指针写进文件

    std::ofstream outfile(mydata::action_id_txt_name); 
    if (!outfile.is_open()) return;
    std::cout << "action_id: " << action_id << std::endl;

    outfile << std::string(action_id) << std::endl;
}

void *on_event(int event_id, void *event_value, size_t event_value_size) {
    return NULL;
}
    

static int g_main_run_ = 1;
char *rkipc_ini_path_ = NULL;
char *rkipc_iq_file_path_ = NULL;


static int zero_count = 0;
// static bool door_closed_reported = false;
static int last_reported_angle = -9999;

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
	gsensor_init();
	qjy_photo_init();
	heat_pwm_init();


    




/*--------------判断图片路径是否存在并创建---------------------*/

    const char *image_tmp_path = "/userdata/tmp_images_path";
    delete_specified_folder(image_tmp_path);
    ensure_path_exists(image_tmp_path);

    const char *images_dir_path = "/userdata/images_dir_path";
    ensure_path_exists(images_dir_path);

	hd_uart_init(addr_biu, "/userdata/images_dir_path", action_id_collect, on_event);

    

	int line_n10 = 10;
    int line_n5 = 5;
 
/*--------------陀螺仪检测并拍照------------------------------*/
    // float angle1 = 20.0f;

    ThreadSafeSet<std::string> photo_names;
    
    
    ThreadSafeSet<std::string> action_id_record;

    bool door_closed_reported = false;

  



    std::thread t1([&image_tmp_path, &images_dir_path, &photo_names, &action_id_record, &door_closed_reported]() {
        while (g_main_run_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            int angle1 = get_angle();
            // std::cout << "***sssssssssssss***: " << angle1 << std::endl;
            float result = tly_detect(angle1);
            // std::cout << "******: " << result << std::endl;

            if (result <= 0) {
                zero_count++;
                // 如果连续10个0以上，并且还没报告过关门
                if (zero_count >= 30 ) {
                    // std::cout << "检测到陀螺仪角度: 0 （确认关门）" << std::endl;
                    // door_closed_reported = true;
                    last_reported_angle = 0;
                }
            } else {
                zero_count = 0;  // 非0则清零计数
                // door_closed_reported = false;
                if (result != last_reported_angle) {
                    std::cout << "检测到陀螺仪角度!!!!!!!!!!: " << result << std::endl;   
                }
                last_reported_angle = result;
            }
          
            if (last_reported_angle >= 10.0f && !door_closed_reported) {

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

               
                
                if (take_photo(2, image_tmp_path,image_biu_name_path)) {
                    std::string image_biu_path = std::string(image_tmp_path) + "/" + image_biu_name_path;
                    photo_names.insert(image_biu_path);
                };

                // writeStringToFileAfterDelay("/userdata/action_id.txt", "hhhhhh", 1);
				// std::this_thread::sleep_for(std::chrono::milliseconds(500));
                last_reported_angle = 1; 
                door_closed_reported = true; // 关门后设置为true，防止重复报告
               
            } else if (last_reported_angle <= 0.0f ) {
                std::string action_id = read_txt_file(mydata::action_id_txt_name);
                // std ::cout << "读取到的action_id: " << action_id << std::endl;
                if (!action_id.empty()) {
                    std::string action_id_image_path_finall = std::string(images_dir_path) + "/" + action_id;
                    ensure_path_exists(action_id_image_path_finall.c_str());
                    movePhotos(photo_names, action_id_image_path_finall, action_id_record);
                    clearFile(mydata::action_id_txt_name);
                    action_id_record.clear();
                } 
           
               door_closed_reported = false;
              
            }
        }
    });





    while (g_main_run_) {
		delete_oldest_folders(images_dir_path);
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




