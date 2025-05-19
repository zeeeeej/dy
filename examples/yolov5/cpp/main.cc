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

#if defined(RV1106_1103) 
    #include "dma_alloc.hpp"
#endif



namespace mydata {
    std::string action_id_txt_name = "/userdata/action_id.txt";
}


void action_id_collect(const unsigned char *action_id, size_t action_id_size){
    std::ofstream outfile(mydata::action_id_txt_name); 
    if (!outfile) return;

    for (size_t i = 0; i < action_id_size; ++i) {
        outfile << std::hex << std::setw(2) << std::setfill('0') << (int)action_id[i];
        if (i != action_id_size - 1) outfile << " ";
    }
    outfile << std::endl;

    outfile.close();
}


int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("%s <model_path> \n", argv[0]);
        return -1;
    }

    // hd_uart_init(action_id_collect);

    const char *model_path = argv[1];
    
/*--------------判断图片路径是否存在并创建---------------------*/
    std::string image_tmp_path = "/userdata/tmp_images";
    ensure_path_exists(image_tmp_path);

    std::string images_dir_path = "/userdata/photo_images";
    ensure_path_exists(images_dir_path);

/*--------------事件图片存储路径------------------------------*/
    std::string action_id_image_path;


/*--------------陀螺仪检测并拍照------------------------------*/
    float angle1 = 20.0f;

    std::set<std::string> photo_names;
    std::thread t1([&angle1, &images_dir_path, &photo_names, &image_tmp_path, &action_id_image_path]() {
        while (true) {
            float result = tly_detect(angle1);
            std::cout << "检测到陀螺仪角度: " << result << std::endl;
          
            if (result >= 0.0f) {
                auto now = std::chrono::system_clock::now();
                // 获取time_t格式（秒）
                std::time_t time_now = std::chrono::system_clock::to_time_t(now);   
                // 线程安全地转换为tm结构
                std::tm tm_now;
                localtime_r(&time_now, &tm_now);
                // 计算毫秒部分
                auto duration = now.time_since_epoch();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
                // 构建路径字符串
                std::ostringstream oss;
                oss << image_tmp_path;
                if (!image_tmp_path.empty() && image_tmp_path.back() != '/')
                    oss << "/";
                
                oss << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S");
                oss << "." << std::setfill('0') << std::setw(3) << millis.count();  // 补零到3位
                oss << ".jpg";
                std::string image_biu_path = oss.str();
                
                if (take_photo(0, image_biu_path)) {
                    photo_names.insert(image_biu_path);
                };
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            } else {
                std::string action_id = read_txt_file(mydata::action_id_txt_name);
                if (!action_id.empty()) {
                    action_id_image_path = images_dir_path + "/" + action_id;
                    movePhotos(photo_names, action_id_image_path);
                    clearFile(mydata::action_id_txt_name);
                } else {
                    std::cout << "开门角度没到阈值!!" << std::endl;
                }
            }
        }
    });
/*--------------创建陀螺仪检测---------------------*/



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
    
        // if (src_image.virt_addr != NULL)
        // {
        // #if defined(RV1106_1103) 
        //         dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
        //                 rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
        // #else
        //         free(src_image.virt_addr);
        // #endif
        //     }  
    }


      


    std::string txt_dir = "/userdata/txt_dir";
    ensure_path_exists(txt_dir);

    

 

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::vector<std::string> frames = get_image_paths(action_id_image_path);
        std::string video_name = "a";
        if (!frames.empty()) {
            video_name = std::filesystem::path(action_id_image_path).filename().string();
            for (const auto& img_path : frames) {
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

                std::string frame_number_str = video_name + "_" + frame_id + "_" + std::to_string(i);


                if (od_results.count == 1 && std::strcmp(coco_cls_to_name(det_result->cls_id), "full")==0 && det_result->prop >= 0.9){
                    std::string txt1_name_path = txt_dir + "/" + video_name + "_1" + ".txt";
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
                    std::string txt2_name_path = txt_dir + "/" + video_name + "_2" + ".txt";
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
        

        std::string txt1_name_path_result = txt_dir + "/" + video_name + "_1" + ".txt";

        std::string txt2_name_path_result = txt_dir + "/" + video_name + "_2" + ".txt";

        std::string crop_img_dirs = "/userdata/crop_images";


        std::string final_result;
        if (file_exists_and_not_empty(txt2_name_path_result)){
            final_result = analyse_two(txt2_name_path_result);
            int line_n = 10;
            std::string crop_img_path = crop_img_dirs + "/" + video_name;
            if (process_last_n_lines(txt2_name_path_result, crop_img_path, line_n)) {
                std::cout << "👍 有截图保存成功" << std::endl;
            } else {
                std::cout << "😢 没有任何截图保存成功" << std::endl;
            }
        }
        if (file_exists_and_not_empty(txt1_name_path_result)) {
            final_result = analyse_one(txt1_name_path_result);
            int line_n = 5;
            std::string crop_img_path = crop_img_dirs + "/" + video_name;
            if (process_last_n_lines(txt1_name_path_result, crop_img_path, line_n)) {
                std::cout << "👍 有截图保存成功" << std::endl;
            } else {
                std::cout << "😢 没有任何截图保存成功" << std::endl;
            }
        }
       
    } 
      
    return 0;
}




