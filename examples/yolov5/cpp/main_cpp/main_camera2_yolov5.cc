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

#include <mutex>

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "rkipc.c"
#include <atomic>


#include "dma_alloc.hpp"


#include <signal.h>
// #include <stdio.h>
// #include <stdlib.h>
#include <dlfcn.h>
#include <unistd.h>

void segfault_handler(int sig, siginfo_t* info, void* ucontext) {
    void* addr = info->si_addr;  // 出错地址
    Dl_info dlinfo;

    if (dladdr(addr, &dlinfo) && dlinfo.dli_fname) {
        fprintf(stderr, "Segmentation fault at address: %p\n", addr);
        fprintf(stderr, "In shared object: %s\n", dlinfo.dli_fname);
        if (dlinfo.dli_sname)
            fprintf(stderr, "Symbol: %s\n", dlinfo.dli_sname);
    } else {
        fprintf(stderr, "Segmentation fault at address: %p (no symbol info)\n", addr);
    }
    _exit(1);  // 直接退出，避免死循环
}

void setup_segv_handler() {
    struct sigaction sa;
    sa.sa_sigaction = segfault_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigaction(SIGSEGV, &sa, NULL);
}




std::string app_version = "V1.1";
int addr_biu = 2;
int pic_id = 65;
static uint8_t door_status = 2;  // 默认日志级别为INFO

static int zero_count = 0;
static int last_reported_angle = -9999;

int width_original = 1080;
int hight_original = 1920;

int move_action = 0;
int line_n10 = 10;
int line_n5 = 5;


static int g_main_run_ = 1;
char *rkipc_ini_path_ = NULL;
char *rkipc_iq_file_path_ = NULL;

namespace mydata {
    std::string action_id_txt_name = "/userdata/action_id.txt";
    std::string status_txt_name = "/userdata/status.txt";
}

// enum { LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG };

// int enable_minilog = 0;
// int rkipc_log_level = LOG_INFO;

void writeStringToFileAfterDelay(const std::string& file_path, const std::string& content, int delay_seconds) {
    
    std::this_thread::sleep_for(std::chrono::seconds(delay_seconds));

 
    std::ofstream outfile(file_path, std::ios::out);
    if (!outfile) {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return;
    }

    outfile << content;
    outfile.close();

    std::cout << "写入完成: " << file_path << std::endl;
}


void *on_event(int event_id, void *event_value, size_t event_value_size) {
    return NULL;
}

void print_meminfo() {
    std::cout << "------ 系统内存状态（MemFree/CMA） ------" << std::endl;

    std::string cmd = "cat /proc/meminfo | grep -i 'memfree\\|cma'";
    std::array<char, 128> buffer;
    FILE* pipe = popen(cmd.c_str(), "r");

    if (!pipe) {
        std::cerr << "popen 失败！" << std::endl;
        return;
    }

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        std::cout << buffer.data();
    }

    pclose(pipe);
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

    // setup_segv_handler();
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
	gsensor_init(1);
	qjy_photo_init();
	heat_pwm_init();

   
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
    const char* model_path = model_path_std.c_str();


    
    // const char *model_path = "/oem/usr/share/one_category_full.rknn";
    
/*--------------判断图片路径是否存在并创建---------------------*/
    const char *image_tmp_path = "/userdata/tmp_images_path";
    const char *images_dir_path = "/userdata/images_dir_path";
    const char *images_original_dir_path = "/userdata/images_oridinal_dir_path";
    const char* txt_dir = "/userdata/txt_dir_path";
    std::string crop_img_dirs = "/userdata/crop_images";

   
    

    remove_folder_if_exists("/userdata/images_dir_path");
    remove_folder_if_exists("/userdata/crop_images");
    remove_folder_if_exists("/userdata/tmp_images_path");
    remove_folder_if_exists("/userdata/txt_dir_path");
    remove_folder_if_exists("/userdata/images_oridinal_dir_path");
   
    ensure_path_exists(image_tmp_path);
    ensure_path_exists(images_dir_path);
    ensure_path_exists(images_original_dir_path);
    ensure_path_exists(crop_img_dirs.c_str());
    ensure_path_exists(txt_dir);
    

    hd_uart_init(addr_biu, crop_img_dirs.c_str(), action_id_collect, on_event);

    std::string final_result;

 


/*--------------陀螺仪检测并拍照------------------------------*/
  
    ThreadSafeSet<std::string> photo_names(20);
    
    
    ThreadSafeSet<std::string> action_id_record(20);

    bool door_closed_reported = false;

    // std::atomic<bool> video_flag(false);

    std::string action_id_add_ = "0";

    
    


    std::thread t1([&image_tmp_path, &images_dir_path, &photo_names, &action_id_record, &images_original_dir_path, &door_closed_reported, &crop_img_dirs, &action_id_add_]() {
        while (g_main_run_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            int angle1 = get_angle();
            float result = tly_detect2(angle1);
         
            if (result <= 0) {
                zero_count++;
                if (zero_count >= 40 ) {
                    last_reported_angle = 0;
                }
                // if (result !=0) {
                //     std::cout << "检测到陀螺仪角度!!!!!!!!!!: " << result << std::endl; 
                // }
               
            } else {
                zero_count = 0;  
                // if (result != last_reported_angle) {
                std::cout << "检测到陀螺仪角度!!!!!!!!!!: " << result << std::endl;   
                // }
                last_reported_angle = result;
            }

            std::string action_id_add = read_txt_file(mydata::action_id_txt_name);

            // if (last_reported_angle >= 20.0f && door_status == 1 && video_flag.load() == true && action_id_add_!= action_id_add) {
            //     action_id_add_ = action_id_add;
            //     std::string crop_img_path = crop_img_dirs + "/" + action_id_add_;
            //     std::filesystem::create_directories(crop_img_path);
            //     createBlankImage(crop_img_path + "/empty_2_1749549124_254.jpg");  // 创建空白图片以避免目录为空
            // }
          
            if (last_reported_angle >= 20.0f && door_status == 1) {

                std::cout << "检测到陀螺仪角度*************: " << last_reported_angle << std::endl;
               
                auto now = std::chrono::system_clock::now();
            
                std::time_t time_now = std::chrono::system_clock::to_time_t(now);   
            
                auto duration = now.time_since_epoch();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
              
                std::ostringstream oss;
             
                // pic_id = (pic_id < 240) ? (pic_id + 1) : 65;

                oss << std::setfill('0') << std::setw(3) << millis.count();  
                // oss << "_" << "2" << "_" << time_now << "_" << pic_id << ".jpg";
                oss << "_" << "2" << "_" << time_now << ".jpg";
                std::string image_biu_name_path = oss.str();

                
                if (take_photo(2, image_tmp_path,image_biu_name_path)) {
                    std::string image_biu_path = std::string(image_tmp_path) + "/" + image_biu_name_path;
                    photo_names.insert(image_biu_path);
                };

                trim_folder_images(image_tmp_path, 20); // 保持临时图片目录最多10张图片

				std::this_thread::sleep_for(std::chrono::milliseconds(200));
                last_reported_angle = 1;   
                door_closed_reported = true;  // 标记门已关闭，避免重复报告
                
               
            } else if (last_reported_angle < 20.0f && door_status == 0 && door_closed_reported) {
                door_closed_reported = false;  
                std::string action_id = read_txt_file(mydata::action_id_txt_name);
                // std ::cout << "读取到的action_id: " << action_id << std::endl;
                clearFile(mydata::action_id_txt_name);
                if (!action_id.empty()) {
                    std::string action_id_image_path_finall = std::string(images_dir_path) + "/" + action_id;
                    ensure_path_exists(action_id_image_path_finall.c_str());
                    movePhotos(photo_names, action_id_image_path_finall, action_id_record);
                    delete_folder_contents_only(image_tmp_path); // 清空临时图片目录
                    copy_folder_to(action_id_image_path_finall, images_original_dir_path);      //*****1111111 */
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

 
   


    while (g_main_run_) {
		delete_oldest_folders(images_dir_path, 5);
        delete_oldest_folders(crop_img_dirs, 50);
        delete_oldest_folders(images_original_dir_path, 5);
       
        std::string action_id_path_biu;
        while (action_id_record.try_pop(action_id_path_biu)) {
        
        // video_flag.store(true);
        
        std::string action_id_biu = std::filesystem::path(action_id_path_biu).filename().string();
        resize_images_in_folder(action_id_path_biu, 960);
        std::vector<std::string> frames = get_image_paths(action_id_path_biu);

/*--------------事件id检测的文件夹------------------------------*/
        std::string txt1_name_path;
        std::string txt2_name_path;
        
        if (!frames.empty()) {
            
            for (const std::string& img_path : frames) {
                    // print_meminfo();
                    std::cout << "图片路径: " << img_path << std::endl;

                    std::string new_path = replace_folder_name_in_path(img_path, "images_dir_path", "images_oridinal_dir_path");

                    cv::Mat image_change = cv::imread(img_path);

                    if (image_change.empty()) {
                        std::cerr << "读取图片失败!" << std::endl;
                        return -1;
                    }

                    int width = image_change.cols;
                    int height = image_change.rows;

                    std::string frame_id = std::filesystem::path(img_path).stem().string();

                    image_buffer_t src_image;
            
                    memset(&src_image, 0, sizeof(image_buffer_t));
                    ret = read_image(img_path.c_str(), &src_image);

                    //RV1106 rga requires that input and output bufs are memory allocated by dma
                    ret = dma_buf_alloc(RV1106_CMA_HEAP_PATH, src_image.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                                    (void **) & (rknn_app_ctx.img_dma_buf.dma_buf_virt_addr));
                    memcpy(rknn_app_ctx.img_dma_buf.dma_buf_virt_addr, src_image.virt_addr, src_image.size);
                    dma_sync_cpu_to_device(rknn_app_ctx.img_dma_buf.dma_buf_fd);
                    free(src_image.virt_addr);
                    src_image.virt_addr = (unsigned char *)rknn_app_ctx.img_dma_buf.dma_buf_virt_addr;
                    src_image.fd = rknn_app_ctx.img_dma_buf.dma_buf_fd;
                    rknn_app_ctx.img_dma_buf.size = src_image.size;
                

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
                        int x1 = det_result->box.left;
                        int y1 = det_result->box.top;
                        int x2 = det_result->box.right;
                        int y2 = det_result->box.bottom;

                    
                        int x1_new = static_cast<int>((static_cast<float>(x1) / width) * width_original);
                        int y1_new = static_cast<int>((static_cast<float>(y1) / height) * hight_original);
                        int x2_new = static_cast<int>((static_cast<float>(x2) / width) * width_original);
                        int y2_new = static_cast<int>((static_cast<float>(y2) / height) * hight_original);


                        std::string frame_number_str = action_id_biu + "_" + frame_id + "_" + std::to_string(i);

        

                        if (od_results.count >= 1 && std::strcmp(coco_cls_to_name(det_result->cls_id), "full")==0 && det_result->prop >= 0.5){
                            txt1_name_path = std::string(txt_dir) + "/" + action_id_biu + "_1" + ".txt";

                            std::ofstream outfile(txt1_name_path, std::ios::app); 
                            if (outfile.is_open()) {
                                outfile << new_path << " "     //***2222222 */
                                        << det_result->cls_id << " "
                                        // << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
                                        << x1_new << " " << y1_new << " " << x2_new << " " << y2_new << " " << det_result->prop <<std::endl;   //***333333 */
                                outfile.close();  // 关闭文件
                                std::cout << "写入成功！" << std::endl;
                            } else {
                                std::cout << "无法打开文件！" << std::endl;
                            }
                        }

                        // if (od_results.count == 2 && std::strcmp(coco_cls_to_name(det_result->cls_id), "full")==0 && det_result->prop >= 0.5){
                        //     std::string txt2_name_path = std::string(txt_dir) + "/" + action_id_biu + "_2" + ".txt";
                        //     // create_empty_txt(txt2_name_path);
                        //     std::ofstream outfile(txt2_name_path, std::ios::app); 
                        //     if (outfile.is_open()) {
                        //         outfile << new_path << " "
                        //                 << det_result->cls_id << " "
                        //                 // << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
                        //                 << x1_new << " " << y1_new << " " << x2_new << " " << y2_new << std::endl;
                        //         outfile.close();  // 关闭文件
                        //         std::cout << "写入成功！" << std::endl;
                        //     } else {
                        //         std::cout << "无法打开文件！" << std::endl;
                        //     }        
                        // }

                    }

                    if (src_image.virt_addr != NULL)
                        {
                            dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                                        rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);                        
                        }  
                    
            }
 
        }


  
        
        if (file_exists_and_not_empty(txt2_name_path)){
           
            // final_result = analyse_two(txt2_name_path);
     
            std::string crop_img_path = crop_img_dirs + "/" + action_id_biu;
        
            if (process_last_n_lines(txt2_name_path, crop_img_path, line_n5, pic_id)) {
                std::cout << "有截图保存成功" << std::endl;
            } else {
                std::cout << "没有任何截图保存成功" << std::endl;
            }
      
            // std::filesystem::path path(txt2_name_path);
            // if (std::filesystem::remove(path)) {
            //     std::cout << "删除成功: " << txt2_name_path << std::endl;
            // } else {
            //     std::cerr << "删除失败: " << txt2_name_path << std::endl;
            // }



        } else {
            // final_result = analyse_one(txt1_name_path);
           
            std::string crop_img_path = crop_img_dirs + "/" + action_id_biu;
          
            if (process_last_n_lines(txt1_name_path, crop_img_path, line_n10, pic_id)) {
                std::cout << "有截图保存成功" << std::endl;
            } else {
                std::cout << "没有任何截图保存成功" << std::endl;
                std::filesystem::create_directories(crop_img_path);
                createBlankImage(crop_img_path + "/empty_2_1749549124_254.jpg");  // 创建空白图片以避免目录为空
            }

            
            // std::filesystem::path path(txt1_name_path);
            // if (std::filesystem::remove(path)) {
            //     std::cout << "删除成功: " << txt1_name_path << std::endl;
            // } else {
            //     std::cerr << "删除失败: " << txt1_name_path << std::endl;
            // }

        }  
       
    }
    // video_flag.store(false);
               
    std::this_thread::sleep_for(std::chrono::seconds(5));
        
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






